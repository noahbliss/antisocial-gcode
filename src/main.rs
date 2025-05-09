use regex::Regex;
use serde::Deserialize;
use static_cell::StaticCell;
use std::fs::File;
use std::io::BufReader;
use std::path::Path;
use std::str::FromStr;
use std::sync::Arc;
use std::time::Duration;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader as AsyncBufReader};
use tokio::net::TcpListener;
use tokio::sync::{broadcast, watch, Mutex};
use tokio::time::sleep;  
use tokio_serial::{SerialPortBuilderExt, SerialStream};

#[derive(Debug, Deserialize, Clone)]
struct Coordinate {
    x: f64,
    y: f64,
}

// #[derive(Debug, Deserialize, Clone)]
// struct TargetMeta {
//     has_x: bool,
//     has_y: bool,
// }

#[derive(Debug, Deserialize)]
struct ExclusionBox {
    top_left: Coordinate,
    top_right: Coordinate,
    bottom_left: Coordinate,
    bottom_right: Coordinate,
}

#[derive(Debug, Deserialize)]
struct Config {
    exclusion_box: ExclusionBox,
}

#[derive(Debug, Clone)]
struct BoxArea {
    xmin: f64,
    xmax: f64,
    ymin: f64,
    ymax: f64,
}

#[derive(Debug, PartialEq)]
enum Side {
    Top,
    Bottom,
    Left,
    Right,
}

/// Reads the JSON configuration file and returns a BoxArea for the exclusion zone.
fn load_config<P: AsRef<Path>>(path: P) -> BoxArea {
    let file = File::open(path).expect("Could not open config file");
    let reader = BufReader::new(file);
    let config: Config = serde_json::from_reader(reader).expect("Error parsing JSON config");

    let xmin = config
        .exclusion_box
        .top_left
        .x
        .min(config.exclusion_box.bottom_left.x);
    let xmax = config
        .exclusion_box
        .top_right
        .x
        .max(config.exclusion_box.bottom_right.x);
    let ymin = config
        .exclusion_box
        .bottom_left
        .y
        .min(config.exclusion_box.bottom_right.y);
    let ymax = config
        .exclusion_box
        .top_left
        .y
        .max(config.exclusion_box.top_right.y);

    BoxArea {
        xmin,
        xmax,
        ymin,
        ymax,
    }
}

/// Returns true if the given point (x, y) is inside the box.
fn point_inside_box(x: f64, y: f64, area: &BoxArea) -> bool {
    x >= area.xmin && x <= area.xmax && y >= area.ymin && y <= area.ymax
}

// /// A rudimentary sampling along the line to see if it passes through the exclusion zone.
// fn line_intersects_box(x0: f64, y0: f64, x1: f64, y1: f64, area: &BoxArea) -> bool {
//     let steps = 400;
//     for i in 0..=steps {
//         let t = (i as f64) / (steps as f64);
//         let x = x0 + t * (x1 - x0);
//         let y = y0 + t * (y1 - y0);
//         if point_inside_box(x, y, area) {
//             return true;
//         }
//     }
//     false
// }

/// Returns the Euclidean distance between two points.
fn distance(a: &Coordinate, b: &Coordinate) -> f64 {
    ((a.x - b.x).powi(2) + (a.y - b.y).powi(2)).sqrt()
}

/// A rudimentary sampling along the line to see if it passes through the exclusion zone.
// fn line_intersects_box(p1: &Coordinate, p2: &Coordinate, area: &BoxArea) -> bool {
//     // fn line_intersects_box(x0: f64, y0: f64, x1: f64, y1: f64, area: &BoxArea) -> bool {
//     let steps = 200;
//     for i in 0..=steps {
//         let t = (i as f64) / (steps as f64);
//         let x = p1.x + t * (p2.x - p1.x);
//         let y = p1.y + t * (p2.y - p1.y);
//         if point_inside_box(x, y, area) {
//             return true;
//         }
//     }
//     false
// }
//

fn line_intersects_box(p1: &Coordinate, p2: &Coordinate, area: &BoxArea) -> bool {
    println!("source: {:?} dest: {:?} area: {:?}", p1, p2, area);
    let mut x1 = p1.x;
    let mut y1 = p1.y;
    let mut x2 = p2.x;
    let mut y2 = p2.y;

    let mut code1 = compute_out_code(x1, y1, area);
    let mut code2 = compute_out_code(x2, y2, area);

    loop {
        if code1 == 0 && code2 == 0 {
            return true; // Both points inside the box
        } else if (code1 & code2) != 0 {
            return false; // Both points outside the box in the same region
        } else {
            let code_out;
            let (mut x, mut y);

            if code1 != 0 {
                code_out = code1;
            } else {
                code_out = code2;
            }

            if (code_out & 8) != 0 {
                x = x1 + (x2 - x1) * (area.ymax - y1) / (y2 - y1);
                y = area.ymax;
            } else if (code_out & 4) != 0 {
                x = x1 + (x2 - x1) * (area.ymin - y1) / (y2 - y1);
                y = area.ymin;
            } else if (code_out & 2) != 0 {
                y = y1 + (y2 - y1) * (area.xmax - x1) / (x2 - x1);
                x = area.xmax;
            } else {
                y = y1 + (y2 - y1) * (area.xmin - x1) / (x2 - x1);
                x = area.xmin;
            }

            if code_out == code1 {
                x1 = x;
                y1 = y;
                code1 = compute_out_code(x1, y1, area);
            } else {
                x2 = x;
                y2 = y;
                code2 = compute_out_code(x2, y2, area);
            }
        }
    }
}

fn compute_out_code(x: f64, y: f64, area: &BoxArea) -> u8 {
    let mut code = 0;

    if x < area.xmin {
        code |= 1;
    } else if x > area.xmax {
        code |= 2;
    }
    if y < area.ymin {
        code |= 4;
    } else if y > area.ymax {
        code |= 8;
    }

    code
}

/// Computes a detour path (as intermediate waypoint(s)) that avoids the exclusion area.
/// Each intermediate waypoint counts as one move and up to two moves are allowed.
/// It always assumes a detour is required (i.e. does not check for a clear direct path).
fn compute_detour_path(start: &Coordinate, target: &Coordinate, area: &BoxArea) -> Vec<Coordinate> {
    let margin = 1.0;

    // Helper to test if a candidate waypoint produces a safe path.
    let valid_waypoint = |wp: &Coordinate| -> bool {
        !line_intersects_box(start, wp, area) && !line_intersects_box(wp, target, area)
    };

    // Attempt one-move detours first.
    let candidates = [
        // Horizontal candidates
        Coordinate {
            x: area.xmin - margin,
            y: target.y,
        },
        Coordinate {
            x: area.xmax + margin,
            y: target.y,
        },
        Coordinate {
            x: area.xmin - margin,
            y: start.y,
        },
        Coordinate {
            x: area.xmax + margin,
            y: start.y,
        },
        // Vertical candidates.
        Coordinate {
            x: target.x,
            y: area.ymin - margin,
        },
        Coordinate {
            x: target.x,
            y: area.ymax + margin,
        },
        Coordinate {
            x: start.x,
            y: area.ymin - margin,
        },
        Coordinate {
            x: start.x,
            y: area.ymax + margin,
        },
    ];

    let mut best_one_move: Option<(Coordinate, f64)> = None;
    for wp in candidates.iter() {
        if valid_waypoint(wp) {
            let total_distance = distance(start, wp) + distance(wp, target);
            if best_one_move
                .as_ref()
                .map_or(true, |&(_, d)| total_distance < d)
            {
                best_one_move = Some((wp.clone(), total_distance));
            }
        }
    }

    if let Some((best_wp, _)) = best_one_move {
        return vec![best_wp];
    }

    // No one-move solution found; try a two-move detour.
    // We consider two strategies: horizontal-first and vertical-first.
    fn two_move_candidate(
        start: &Coordinate,
        target: &Coordinate,
        area: &BoxArea,
        margin: f64,
        horizontal_first: bool,
    ) -> Option<(Vec<Coordinate>, f64)> {
        if horizontal_first {
            // First move: horizontal shift from the start to a safe X.
            let safe_x = if start.x < area.xmin {
                area.xmin - margin
            } else if start.x > area.xmax {
                area.xmax + margin
            } else {
                // When start is horizontally in range, prefer left side.
                area.xmin - margin
            };
            let wp1 = Coordinate {
                x: safe_x,
                y: start.y,
            };
            // Second move: vertical adjustment to a safe Y near the target.
            let safe_y = if target.y > area.ymax {
                area.ymax + margin
            } else if target.y < area.ymin {
                area.ymin - margin
            } else {
                // When target is within the vertical bounds, choose the closer edge relative to start.
                if (start.y - (area.ymin - margin)).abs() < (start.y - (area.ymax + margin)).abs() {
                    area.ymin - margin
                } else {
                    area.ymax + margin
                }
            };
            let wp2 = Coordinate {
                x: safe_x,
                y: safe_y,
            };
            if !line_intersects_box(start, &wp1, area)
                && !line_intersects_box(&wp1, &wp2, area)
                && !line_intersects_box(&wp2, target, area)
            {
                let total_distance =
                    distance(start, &wp1) + distance(&wp1, &wp2) + distance(&wp2, target);
                return Some((vec![wp1, wp2], total_distance));
            }
        } else {
            // Vertical-first: first move from start vertically to a safe Y.
            let safe_y = if start.y < area.ymin {
                area.ymin - margin
            } else if start.y > area.ymax {
                area.ymax + margin
            } else {
                area.ymin - margin
            };
            let wp1 = Coordinate {
                x: start.x,
                y: safe_y,
            };
            // Second move: horizontal adjustment to a safe X near the target.
            let safe_x = if target.x > area.xmax {
                area.xmax + margin
            } else if target.x < area.xmin {
                area.xmin - margin
            } else {
                if (start.x - (area.xmin - margin)).abs() < (start.x - (area.xmax + margin)).abs() {
                    area.xmin - margin
                } else {
                    area.xmax + margin
                }
            };
            let wp2 = Coordinate {
                x: safe_x,
                y: safe_y,
            };
            if !line_intersects_box(start, &wp1, area)
                && !line_intersects_box(&wp1, &wp2, area)
                && !line_intersects_box(&wp2, target, area)
            {
                let total_distance =
                    distance(start, &wp1) + distance(&wp1, &wp2) + distance(&wp2, target);
                return Some((vec![wp1, wp2], total_distance));
            }
        }
        None
    }

    let candidate1 = two_move_candidate(start, target, area, margin, true);
    let candidate2 = two_move_candidate(start, target, area, margin, false);

    let best_two = match (candidate1, candidate2) {
        (Some((path1, dist1)), Some((path2, dist2))) => {
            if dist1 <= dist2 {
                Some(path1)
            } else {
                Some(path2)
            }
        }
        (Some((path, _)), None) => Some(path),
        (None, Some((path, _))) => Some(path),
        (None, None) => None,
    };

    // If no valid two-move solution was found (unexpected), return an empty detour.
    let mut path = best_two.unwrap_or_default();
    // path.push(target.clone());
    return path;
}

/// Computes a path that skirts the exclusion area and then enters it
/// through the boundary (left or right) that is closest to the target.
/// If the direct route from start to target is already in compliance with
/// the “entry rule” (i.e. entering from the proper boundary), then unnecessary
/// intermediate moves are avoided.
fn compute_entry_path_via_boundary(
    start: &Coordinate,
    target: &Coordinate,
    area: &BoxArea,
) -> Vec<Coordinate> {
    // let eps = 0.001; // tolerance for alignment

    // Determine if the target is on the left or right half of the exclusion area.
    let area_mid_x = (area.xmin + area.xmax) / 2.0;
    // Choose the appropriate boundary and offset values.
    // For left entry, we approach from outside the left side (offset -1.0).
    // For right entry, we approach from outside the right side (offset +1.0).
    let (entry_boundary, offset, x_side) = if target.x < area_mid_x {
        (area.xmin, -1.0, Side::Left)
    } else {
        (area.xmax, 1.0, Side::Right)
    };

    // Compute the ideal external x position (outside the box) for entry.
    let ideal_x = entry_boundary + offset;

    // Compute the vertical waypoint y coordinate.
    let area_mid_y = (area.ymin + area.ymax) / 2.0;
    let ideal_y = target.y;

    let mut path = Vec::new();

    let corner_wp_needed = if x_side == Side::Left && start.x <= area.xmin {
        false
    } else if x_side == Side::Right && start.x >= area.xmax {
        false
    } else {
        true
    };
    println!(
        "Entering from {:?} with start.x of {} - corner needed: {}",
        x_side, start.x, corner_wp_needed
    );

    let safe_top_left = Coordinate {
        x: area.xmin - 1.0,
        y: area.ymax + 1.0,
    };
    let safe_bottom_left = Coordinate {
        x: area.xmin - 1.0,
        y: area.ymin - 1.0,
    };
    let safe_top_right = Coordinate {
        x: area.xmax + 1.0,
        y: area.ymax + 1.0,
    };
    let safe_bottom_right = Coordinate {
        x: area.xmax + 1.0,
        y: area.ymin - 1.0,
    };

    if !corner_wp_needed {
        if start.y >= area.ymax && start.y <= area.ymin {
            let hop1 = Coordinate {
                x: ideal_x,
                y: ideal_y,
            };
            path.push(hop1);
        }
        // path.push(target.clone());
        return path;
    } else if x_side == Side::Left {
        // We're aligned with the exclusion area on the wrong side.
        match start.y > area_mid_y {
            true => {
                if start.y < area.ymax && start.y > area.ymin {
                    path.push(safe_top_right);
                }
                path.push(safe_top_left);
            }
            false => {
                if start.y < area.ymax && start.y > area.ymin {
                    path.push(safe_bottom_right);
                }
                path.push(safe_bottom_left);
            }
        };
        let pre_enter = Coordinate {
            x: ideal_x,
            y: ideal_y,
        };
        path.push(pre_enter);
    } else {
        // side right
        match start.y > area_mid_y {
            true => {
                if start.y < area.ymax && start.y > area.ymin {
                    path.push(safe_top_left);
                }
                path.push(safe_top_right);
            }
            false => {
                if start.y < area.ymax && start.y > area.ymin {
                    path.push(safe_bottom_left);
                }
                path.push(safe_bottom_right);
            }
        };
        let pre_enter = Coordinate {
            x: ideal_x,
            y: ideal_y,
        };
        path.push(pre_enter);
    }

    // Finally, add the target which is assumed to be inside the exclusion area.
    // path.push(target.clone());

    // If no intermediate waypoints were added (or only one was added) and the straight
    // line from start to target already "enters" via the desired boundary, then the path
    // will simply be [start, target] (or simply [target] if start is considered the
    // current position). Here we return the intermediate waypoints; callers can then prepend
    // the start if needed.
    path
}

/// Given a list of points, create GCODE (G1) commands that move to each point.
fn create_gcode_for_path(path: &[Coordinate], s: Option<f64>, f: Option<f64>) -> Vec<String> {
    let s = s.unwrap_or(800 as f64);
    let f = f.unwrap_or(12000 as f64);
    path.iter()
        //M204S488.78G1X4.0008Y86.8615F3820.03
        .map(|pt| format!("M204 S{} G1 X{:.3} Y{:.3} F{}", s, pt.x, pt.y, f))
        .collect()
}

/// Process a single GCODE command.
/// If it's a motion command (G0 or G1 with X & Y), it may be modified if a detour is needed.
/// The output commands are written to the provided serial writer.
async fn process_gcode_command(
    command: &str,
    current_pos: &mut Coordinate,
    exclusion_area: &BoxArea,
    serial_writer: &mut (impl AsyncWriteExt + Unpin),
    gcode_regex: &Regex,
    detour_signal: &tokio::sync::watch::Sender<bool>,
) -> anyhow::Result<()> {
    let trimmed = command.trim();
    // let mut target_meta = TargetMeta {
    //     has_x: false,
    //     has_y: false,
    // };
    println!("Received GCODE: {}", trimmed);

    if trimmed.eq("G28") {
        println!("Received G28 home command");
        *current_pos = Coordinate { x: 0.0, y: 0.0 };
        return Ok(());
    }

    if let Some(captures) = gcode_regex.captures(trimmed) {
        // Extract target S, X, Y, and F.

        let target_s = captures
            .get(2)
            .map(|m| f64::from_str(m.as_str()).ok())
            .unwrap_or(None);
        let target_x = captures
            .get(3)
            .map(|m| f64::from_str(m.as_str()).unwrap_or(current_pos.x))
            .unwrap();
        let target_y = captures
            .get(4)
            .map(|m| f64::from_str(m.as_str()).unwrap_or(current_pos.y))
            .unwrap();
        let target_f = captures
            .get(5)
            .map(|m| f64::from_str(m.as_str()).ok())
            .unwrap_or(None);
        let target = Coordinate {
            x: target_x,
            y: target_y,
        };

        if point_inside_box(target.x, target.y, exclusion_area) {
            if point_inside_box(current_pos.x, current_pos.y, exclusion_area) {
                println!("Target is inside exclusion zone and so are we. Moving directly.");
                serial_writer
                    .write_all(format!("{}\n", trimmed).as_bytes())
                    .await?;
                *current_pos = target;
                return Ok(());
            } else {
                println!("Entering the exclusion zone, routing via a safe boundary...");
                let mut entry_path =
                    compute_entry_path_via_boundary(current_pos, &target, &exclusion_area);
                entry_path.push(target.clone()); //I know this is redundant but it is easier than double parsing the original command.
                let entry_commands = create_gcode_for_path(&entry_path, target_s, target_f);
                for cmd in entry_commands {
                    serial_writer
                        .write_all(format!("{}\n", cmd).as_bytes())
                        .await?;
                }
                serial_writer
                    .write_all(format!("{}\n", trimmed).as_bytes())
                    .await?;
                *current_pos = target;
                return Ok(());
            }
        }

        // If we are already inside the exclusion zone and want to leave, bypass the detour logic.
        if point_inside_box(current_pos.x, current_pos.y, exclusion_area) {
            println!(
                "Currently inside exclusion zone. Leaving zone via closest Y boundary to target."
            );

            let mut path = Vec::new();

            if target.y >= ((exclusion_area.ymax + exclusion_area.ymin) / 2.0) {
                //go up
                path.push(Coordinate {
                    x: current_pos.x,
                    y: (exclusion_area.ymax + 1.0),
                });
            } else {
                //go down
                path.push(Coordinate {
                    x: current_pos.x,
                    y: (exclusion_area.ymin + 1.0),
                });
            }

            // path.push(target.clone());
            path.push(target.clone()); //I know this is redundant but it is easier than double parsing the original command.
            let exit_commands = create_gcode_for_path(&path, target_s, target_f);
            for cmd in exit_commands {
                serial_writer
                    .write_all(format!("{}\n", cmd).as_bytes())
                    .await?;
            }
            serial_writer
                .write_all(format!("{}\n", trimmed).as_bytes())
                .await?;
            *current_pos = target;
            return Ok(());
        }

        if line_intersects_box(current_pos, &target, exclusion_area) {
            println!("Direct move crosses exclusion zone. Computing detour…");
            detour_signal.send(true).unwrap();
            let mut detour_points = compute_detour_path(current_pos, &target, exclusion_area);
            detour_points.push(target.clone()); //I know this is redundant but it is easier than double parsing the original command.
            let detour_commands = create_gcode_for_path(&detour_points, target_s, target_f);
            for cmd in detour_commands {
                println!("Sending detour command: {}", cmd);
                serial_writer
                    .write_all(format!("{}\n", cmd).as_bytes())
                    .await?;
            }
            serial_writer
                .write_all(format!("{}\n", trimmed).as_bytes())
                .await?;
            detour_signal.send(false).unwrap();
            *current_pos = target;
        } else {
            println!("Direct move valid. Sending command.");
            serial_writer
                .write_all(format!("{}\n", trimmed).as_bytes())
                .await?;
            *current_pos = target;
        }
    } else {
        // Pass through non-motion commands.
        println!("Non-motion command. Passing through.");
        serial_writer
            .write_all(format!("{}\n", trimmed).as_bytes())
            .await?;
    }
    Ok(())
}

/// Handles a TCP connection from a client.
/// In addition to processing incoming GCODE commands from the client,
/// this function also spawns a task which subscribes to serial responses and
/// writes them back to the network client.
async fn handle_client(
    stream: tokio::net::TcpStream,
    exclusion_area: BoxArea,
    serial_writer: Arc<Mutex<tokio::io::WriteHalf<SerialStream>>>,
    gcode_regex: Regex,
    current_pos: Arc<Mutex<Coordinate>>,
    mut serial_rx: broadcast::Receiver<String>,
) {
    let peer = stream.peer_addr().expect("Could not get peer address");
    println!("Accepted connection from {}", peer);

    // Split the TCP stream into reader and writer.
    let (reader, writer) = stream.into_split();
    // Wrap the TCP writer in an Arc/Mutex so it can be shared; this writer is used solely for sending
    // serial responses to the client.
    let writer = Arc::new(Mutex::new(writer));
    let buf_reader = AsyncBufReader::new(reader);
    let mut lines = buf_reader.lines();

    // Spawn a task to forward serial responses from the broadcast channel to the TCP client.
    let writer_clone = Arc::clone(&writer);
    let (detouring_w, mut detouring_r) = watch::channel(false);
    tokio::spawn(async move {
        loop {
            match serial_rx.recv().await {
                Ok(line) => {
                    if *detouring_r.borrow_and_update() == true {
                        println!("Waiting for detour to complete...");
                        detouring_r.changed().await.unwrap();
                        println!("Detour completed, resuming exgress messages.");
                    }
                    let mut writer_lock = writer_clone.lock().await;
                    if let Err(e) = writer_lock
                        .write_all(format!("{}\n", line).as_bytes())
                        .await
                    {
                        eprintln!("Failed to send serial response to client: {:?}", e);
                        break;
                    }
                }
                Err(broadcast::error::RecvError::Lagged(count)) => {
                    eprintln!("Client lagged {} messages", count);
                }
                Err(broadcast::error::RecvError::Closed) => break,
            }
        }
    });

    // Process commands from the client.
    while let Ok(Some(line)) = lines.next_line().await {
        if line.trim().is_empty() {
            continue;
        }

        {
            let mut pos_lock = current_pos.lock().await;
            let mut serial_lock = serial_writer.lock().await;
            if let Err(e) = process_gcode_command(
                &line,
                &mut *pos_lock,
                &exclusion_area,
                &mut *serial_lock,
                &gcode_regex,
                &detouring_w,
            )
            .await
            {
                eprintln!("Error processing command: {:?}", e);
            }
        }
    }
    println!("Connection from {} closed", peer);
    return;
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    // Load the exclusion zone configuration.
    let exclusion_area = load_config("exclusion_config.json");
    println!("Loaded exclusion zone: {:?}", exclusion_area);

    // Open the serial port at /dev/ttyACM0.
    let serial_port_name = "/dev/ttyACM0";
    let baud_rate = 115200;
    let serial = tokio_serial::new(serial_port_name, baud_rate)
        .timeout(Duration::from_millis(1000))
        .open_native_async()?;
    println!(
        "Opened serial port {} at {} baud.",
        serial_port_name, baud_rate
    );

    // Split the serial port into a reader and writer.
    let (serial_reader, serial_writer) = tokio::io::split(serial);
    let serial_writer = Arc::new(Mutex::new(serial_writer));

    // Wrap the current position in an Arc/Mutex to share between tasks.
    let current_pos = Arc::new(Mutex::new(Coordinate { x: 0.0, y: 0.0 }));

    // Precompile a regex to match G0/G1 commands with X and Y coordinates.
    let gcode_regex = Regex::new(r"^(G0|G1|M204)(?:.*?S([-+]?[0-9]*\.?[0-9]+))?(?:.*?X([-+]?[0-9]*\.?[0-9]+))?(?:.*?Y([-+]?[0-9]*\.?[0-9]+))?(?:.*?F([-+]?[0-9]*\.?[0-9]+))?")
        .expect("Failed to compile regex");

    // Create a broadcast channel to carry serial responses.
    let (serial_tx, _) = broadcast::channel::<String>(100);
    let serial_tx_sub = serial_tx.clone();

    // Spawn a task to continuously read lines from the serial port and forward them via the broadcast channel.
    tokio::spawn(async move {
        let mut reader = AsyncBufReader::new(serial_reader);
        let mut line = String::new();
        loop {
            line.clear();
            match reader.read_line(&mut line).await {
                Ok(0) => {
                    eprintln!("Serial port closed.");
                    break;
                }
                Ok(_) => {
                    let trimmed = line.trim().to_string();
                    println!("Received from serial: {}", trimmed);
                    if !trimmed.is_empty() {
                        let _ = serial_tx.send(trimmed);
                    }
                }
                Err(e) => {
                    eprintln!("Error reading from serial: {:?}", e);
                    break;
                }
            }
        }
    });

    // Bind a TCP listener on localhost port 23.
    let listener = TcpListener::bind("127.0.0.1:23").await?;
    println!("Listening on 127.0.0.1:23 for incoming GCODE connections...");

    loop {
        let (socket, _) = listener.accept().await?;
        let exclusion_area_clone = exclusion_area.clone();
        let gcode_regex_clone = gcode_regex.clone();
        let serial_writer_clone = Arc::clone(&serial_writer);
        let current_pos_clone = Arc::clone(&current_pos);
        // For each new client, create a subscription to the broadcast channel.
        let serial_rx = serial_tx_sub.subscribe();

        tokio::spawn(async move {
            handle_client(
                socket,
                exclusion_area_clone,
                serial_writer_clone,
                gcode_regex_clone,
                current_pos_clone,
                serial_rx,
            )
            .await;
        });
    }
}

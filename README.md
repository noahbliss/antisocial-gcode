# Antisocial Gcode

This is a gcode proxy written in rust that denotes a keepout rectangle for a given machine. It listens on port 23 and interfaces with a downstream serial device. My primary use-case is to prevent colisions with a nozzle rack with openpnp, and this code is a minimum viable product for my use-case.

When a movement gcode command is received, it will check if the movement would intersect with the keepout rectangle. If it does, it injects commands to route around the keepout rectangle.

This code is poorly optimized, and highly opinionated. You will want to adjust the regex and command injection portion to reflect your own needs, specifically as it relates to speed. An example exclusion file is provided.

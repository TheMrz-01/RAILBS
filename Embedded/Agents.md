## Agents.md

This repo is a codebase for a Istanbul Metropolitan Municipality Rail Vehicle competition.
The vehicle consists of two rs550 dc motors controlled with two BTS7960B motor drivers connected to a esp32.
For magnet detection it uses dual KY-024. Placed upfront and also back of the vehicle and for tunnel detection it will use HC-SR04 sensor.The rest is spesification for track, rules, constraits, and scoring formula.

# Competition Constraints and Requirements

This robot is being developed for the Metro Istanbul Rail System Vehicles Competition, organized by Loth Robotics and Beşiktaş Sakıp Sabancı Anatolian High School in cooperation with Metro Istanbul.

## Competition Objective

The robot must autonomously travel on a 20-meter rail track, stop inside a tunnel for 5 seconds, then continue to the finish line and stop as close as possible to the target point.

The final task score depends on:

- Total mission completion time
- Distance from the finish line after stopping
- Accuracy of the 5-second tunnel stop

## Track Specifications

- The track consists of two parallel aluminum box-profile rails.
- Rail gauge: `180 mm`
- Total competition distance: `20 m`
- Rail profile dimensions:
  - Width: `17 mm`
  - Height: `25 mm`
  - Section length: `3 m`
- Track tolerances may vary by approximately `±3 mm`.
- The robot must be mechanically tolerant to small gaps, misalignments, and vibrations between rail sections.
- Derailment prevention is important; wheel geometry should safely keep the robot on the rails.

## Traverses and Magnetic Markers

- Traverses are placed every `50 cm` along the track.
- Each traverse contains one magnet:
  - Diameter: `10 mm`
  - Thickness: `5 mm`
- The top of the magnet is level with the aluminum rails.
- The start and finish positions also include magnet-containing traverses.
- The finish line is considered the center of the 41st traverse, including the starting traverse.

These magnets may be used for localization and position estimation.

## Tunnel Specifications

- The tunnel is located at approximately `9.12 m` from the start.
- Tunnel length: `88 cm`
- Tunnel height: `30 cm`
- Tunnel wall thickness: `2 mm`
- A presence sensor is mounted on the tunnel ceiling at the `50 cm` point of the tunnel.
- The robot must stop inside the tunnel for `5 seconds`.

## Mission Procedure

The robot starts from the initial traverse. Before the mission, judges may place the robot at any point on the `150 mm` long starting traverse.

Mission sequence:

1. Wait for judge approval.
2. Power on the robot.
3. Wait for the signal.
4. Switch the robot to autonomous mode.
5. Receive the start command from the judge.
6. The robot completes the mission autonomously.

The mission timer starts when the team member presses the button after judge approval and ends when the robot has completely stopped.

## Vehicle Technical Constraints

The robot must satisfy the following technical rules:

- The robot may only use electric power.
- The robot must safely stand on the rail using at least 4 wheels.
- Maximum mass: `2 kg`
- Maximum length: `1 m`
- The battery must be unmodified and its brand must be visible.
- The robot must include a current cutoff switch and a fuse connected to one terminal of the battery.
- A fused switch may be used instead of separate fuse and switch components.
- The current cutoff switch must:
  - Be fixed to the robot
  - Be on an accessible surface
  - Be reachable within at most `3 seconds`

## Control and Autonomy Restrictions

The competition prioritizes innovative driving and localization algorithms.

The use of systems that calculate distance from rotation count is discouraged, including:

- Encoders
- Encoder motors
- Stepper motors
- Similar rotation-count-based odometry systems

If such systems are used, they must be declared in the design report.

If the team uses these systems without declaring them, the software score from the design report may be ignored in the final score calculation.

During the official mission run:

- Wired or wireless remote controllers are not allowed.
- A wireless computer may only be used to send the initial driving/start command.
- After the start command, team members may not interfere with the robot.

## Scoring Formula

Task score is calculated using:

```text
100 - [4x + 2(t - 10)] - min(|y - 5|, 5)
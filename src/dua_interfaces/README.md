# dua_interfaces

ROS 2 interfaces for the Distributed Unified Architecture, aimed at robotics projects and more.

## Contents

Refer to the individual interface files for more information.

### Messages

- [x] [`CommandResult`](msg/CommandResult.msg): Result of a command execution, useful to be included in high-level communications like services and actions.
- [x] [`CommandResultStamped`](msg/CommandResultStamped.msg): Result of a command execution, with a timestamp.
- [x] [`CoordinateSystem`](msg/CoordinateSystem.msg): Coordinate system, used to represent the reference frame of a pose (mainly useful as an enum).
- [x] [`EulerAttitudeSetpoint`](msg/EulerAttitudeSetpoint.msg): Setpoint for an Euler attitude controller.
- [x] [`EulerPoseStamped`](msg/EulerPoseStamped.msg): Pose with Euler angles, avoids the need for a quaternion to Euler conversion.
- [x] [`PositionSetpoint`](msg/PositionSetpoint.msg): Setpoint for a position controller.
- [x] [`VelocitySetpoint`](msg/VelocitySetpoint.msg): Setpoint for a velocity controller.

## Requirements

Builds on ROS 2 Humble Hawksbill.

See [`package.xml`](package.xml) for more information.

---

## License

This work is licensed under the GNU General Public License v3.0. See the [`LICENSE`](LICENSE) file for details.

## Copyright

Copyright (c) 2023, Intelligent Systems Lab, University of Rome Tor Vergata

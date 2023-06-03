# dua_interfaces

ROS 2 interfaces for the Distributed Unified Architecture, aimed at robotics projects and more.

## Contents

Refer to the individual interface files for more information.

### Messages

- [x] [`CommandResultStamped`](msg/CommandResultStamped.msg): Result of a command execution, useful to be included in high-level communications like services and actions, with a ROS header.
- [x] [`EulerAttitudeSetpoint`](msg/EulerAttitudeSetpoint.msg): Setpoint for an Euler attitude controller.
- [x] [`EulerPoseStamped`](msg/EulerPoseStamped.msg): Pose with Euler angles, avoids the need for a quaternion to Euler conversion.
- [x] [`PointCloud2WithROI`](msg/PointCloud2WithROI.msg): Point cloud with a region of interest.
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

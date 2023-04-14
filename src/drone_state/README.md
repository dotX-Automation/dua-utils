# drone_state

C++ library based on [`Eigen`](http://eigen.tuxfamily.org/index.php?title=Main_Page) for representing the state of a drone in 2D/3D space, for fast and efficient computations.

## Contents

Three classes:

- `Pose`: position and orientation,
- `KinematicPose`: position, orientation and linear/angular velocity.
- `DynamicPose`: position, orientation, linear/angular velocity and linear/angular acceleration.

### Features

All classes are based on the `double` type, and offer:

- Various constructors for different use cases and data availability.
- Representation of attitude as both Euler angles and quaternions.
- Full compatibility with fixed-size Eigen geometric types.
- Getters and setters.
- Assigment operators, copy and move constructors.
- Compatibility with ROS 2 messages from the [`dua_interfaces`](../dua_interfaces/README.md) package.
- Integration with the `ament` build system.

### Geometric operations

Currently implemented and planned:

- [x] Supported reference frames conversions.
- [x] Roto-translation (as `operator*`).

## Requirements

Builds on ROS 2 Humble Hawksbill.

See [`package.xml`](package.xml) for more information.

---

## License

This work is licensed under the GNU General Public License v3.0. See the [`LICENSE`](LICENSE) file for details.

## Copyright

Copyright (c) 2023, Intelligent Systems Lab, University of Rome Tor Vergata

# dua_qos_py

Collection of getter functions that return common ROS 2 Quality of Service (QoS) profiles for DUA Python modules.

## Contents

This library contains many different QoS profile getters. Such profiles are intended for **topics**, since services and actions have their own QoS profiles; those could be modified too, but handling the many different possible repercussions of such a change on their client-server, stateful communications is beyond the scope of this library.

Topics are divided into two categories, which correspond to the two main types of transmission in a ROS 2 network and to two distinct modules in the library:

- **Data topics**, which are critical for the correct functioning of the system and are intended to be used for the transmission of data. They correspond to the `dua_qos` module.
- **Visualization topics**, which are not critical at all and are only intended to be used for inspection purposes, by tools such as RViz 2. They correspond to the `dua_qos_visualization` module.

The contents of each module are described below. Each getter takes an additional argument which is the *depth* of the QoS profile, *i.e.*, the maximum number of samples that can be stored in the history of the topic. This is useful to avoid memory leaks in case of a slow consumer, but it is not always necessary to set it to a value greater than 1. There are default values, which are suggested in most situations.

### `dua_qos`

- `get_datum_qos`: Returns a QoS profile for a data topic that is critical for the correct functioning of the system, so reliable by default.
- `get_command_qos`: Returns a QoS profile for a command topic that is critical for the correct functioning of the system, so reliable by default.
- `get_scan_qos`: Returns a QoS profile for a spatial scan topic, *e.g.*, laser scan, pointcloud, or map, that must be reliable but with a low depth because of their potential size.
- `get_image_qos`: Returns a QoS profile for an image topic, *e.g.*, camera image; reliable transmission of large messages such as images is very bandwidth-intensive and could clog a DDS network over a lossy link (*i.e.*, WiFi), so use this with care.

### `dua_qos_visualization`

These getters all have a best-effort reliability policy, since visualization topics are not critical for the correct functioning of the system and their loss should not be a problem, whereas they should not take up too much bandwidth.

- `get_datum_qos`: Returns a QoS profile for a data topic.
- `get_command_qos`: Returns a QoS profile for a command topic.
- `get_scan_qos`: Returns a QoS profile for a spatial scan topic, *e.g.*, laser scan, pointcloud, or map.
- `get_image_qos`: Returns a QoS profile for an image topic, *e.g.*, camera image.

---

## License

This work is licensed under the GNU General Public License v3.0. See the [`LICENSE`](LICENSE) file for details.

## Copyright

Copyright (c) 2023, Intelligent Systems Lab, University of Rome Tor Vergata

# simple_serviceclient

C++ library that wraps the `rclcpp` service client providing a simpler interface.

## Contents

This package offers the `SimpleServiceClient::Client<ServiceT>` class template, that can be used to create a ROS 2 service client just by passing the node and the service name.

### Features

The service can be called in two different fashions:

- `call_sync`: synchronous call, that blocks until the service response is received but spins the node in the meantime, returning the response.
- `call_async`: asynchronous call, that returns a `FutureAndRequestId` object that can be used to retrieve the response when it is received.

---

## License

This work is licensed under the GNU General Public License v3.0. See the [`LICENSE`](LICENSE) file for details.

## Copyright

Copyright (c) 2023, Intelligent Systems Lab, University of Rome Tor Vergata

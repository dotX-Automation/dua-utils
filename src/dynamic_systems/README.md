# filters

C++ library to implement dynamic systems as ROS 2 plugins. Requires `pluginlib`.

## Contents

This is a collection made of the following ROS 2 packages.

### `dynamic_system_base`

This is the base package for all dynamic systems.

`DynamicSystems::DynamicSystem` is an abstract class that follows the requirements of the `pluginlib` library, so it can only be used to derive new dynamic systems.

`DynamicSystems::InitParams` is a struct that contains the initialization parameters of the dynamic systems. It is used to initialize the dynamic systems, and should be derived to contain the parameters of the specific dynamic systems. By default, it contains the sampling time and the number of discretization steps used for zero-order-hold discretization.

`DynamicSystems::SetupParams` is a struct that contains the setup parameters of the dynamic systems. It is used to change the dynamic systems' behaviour, and should be derived to contain the parameters of the specific dynamic systems. By default, it is empty.

The following interface is provided and must be followed in new implementations:

- `void DynamicSystems::DynamicSystem::init(std::shared_ptr<InitParams> params)`: initializes the filter with the given parameters, taken as a shared pointer since you would initialize them as such using `pluginlib`'s class loaders. **This is necessary since `pluginlib` does not support constructors with parameters, nor copy or move semantics.**
- `void DynamicSystems::DynamicSystem::setup(std::shared_ptr<SetupParams> params)`:
- `void DynamicSystems::DynamicSystem::reset()`: resets the filter to its initial state.
- `void DynamicSystems::DynamicSystem::input()`:
- `void DynamicSystems::DynamicSystem::output()`:
- `void DynamicSystems::DynamicSystem::step()`:
- `void DynamicSystems::DynamicSystem::evolve()`: updates the filter with the given new sample, but **does not return the output**.
- `void DynamicSystems::DynamicSystem::fini()`: finalizes the filter, freeing any allocated memory, etc..
- `void DynamicSystems::DynamicSystem::~Filter()`: destructor.

### `dynamic_system_plugins`

This package contains implementations of dynamic systems as plugins.

Currently, the following dynamic systems are available:

- [x] `PID`: implements a PID controller.

---

## License

This work is licensed under the GNU General Public License v3.0. See the [`LICENSE`](../../LICENSE) file for details.

## Copyright

Copyright (c) 2023, Intelligent Systems Lab, University of Rome Tor Vergata

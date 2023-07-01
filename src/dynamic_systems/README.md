# dynamic_systems

C++ library to implement modular dynamic systems. Requires `eigen 3.4`.

## Contents

This is a collection made of the following ROS 2 packages.

### `dynamic_system_base`

This is the base package for all dynamic systems.

`DynamicSystems::InitParams` is a struct that contains the initialization parameters of the dynamic systems. It is used to initialize the dynamic systems, and should be derived to contain the parameters of the specific dynamic systems. By default, it is empty.

`DynamicSystems::SetupParams` is a struct that contains the setup parameters of the dynamic systems. It is used to change the dynamic systems' behaviour, and should be derived to contain the parameters of the specific dynamic systems. By default, it is empty.

`DynamicSystems::State` is a struct that contains the state of the dynamic systems. It is used to retain the whole state of the dynamic systems, and should be derived to contain the internal variables of the specific dynamic systems. By default, it is empty.

`DynamicSystems::System` is an virtual class that follows the requirements of the `pluginlib` library, so it can only be used to derive new dynamic systems.

The specialization of the structures `DynamicSystems::InitParams`, `DynamicSystems::SetupParams`, and `DynamicSystems::State` shall provide deep copy functionality throught `copy` and `clone` methods.

The virtual class `DynamicSystems::System` offers the following public methods:

- `void DynamicSystems::System::init(std::shared_ptr<InitParams> initParams)`: initializes the dynamic system with the given parameters.
- `void DynamicSystems::System::setup(std::shared_ptr<SetupParams> setupParams)`: change the dynamic system behaviour with the given parameters.
- `bool DynamicSystems::System::fini()`: finalizes the filter, freeing any allocated memory, etc..
- `bool DynamicSystems::System::initialized()`: whether the system is initialized..
- `void DynamicSystems::System::dirty()`: whether the output of the system must be updated..
- `bool DynamicSystems::System::reset(std::shared_ptr<State> state = nullptr)`: resets the filter to last provided resetting state.
- `bool DynamicSystems::System::input(MatrixXd in)`: insert input inside the system.
- `MatrixXd DynamicSystems::System::output()`: get output from the system. If dirty, update the output of the system before return.
- `void DynamicSystems::System::step()`: update system's internal state.
- `void DynamicSystems::System::update()`: update the output of the system, if dirty.
- `MatrixXd DynamicSystems::System::evolve(MatrixXd in)`: execute in order `input`, `output` and `step`.

This class also offers the following protected interface (to be specialized by derived dynamic systems):

- `void DynamicSystems::System::init_parse(const InitParams& initParams)`: parse the given parameters and initialize the system.
- `void DynamicSystems::System::setup_parse(const SetupParams& setupParams)`: parse the given parameters and set system's internal parameters.
- `void DynamicSystems::System::setup_default()`: set default values for system's internal parameters configuration.
- `void DynamicSystems::System::deinit()`: release internal structures and allocated memory used by the derived system.
- `void DynamicSystems::System::state_validator(State &state)`: validate and eventually change system's internal status at every change.
- `void DynamicSystems::System::input_validator(const State &state, MatrixXd &input)`: validate and eventually change the passed input eventually considering the internal state.
- `void DynamicSystems::System::dynamic_map(const State &state, const MatrixXd &input, State &next)`: specify the dynamic behaviour of the specified system.
- `void DynamicSystems::System::output_map(const State &state, const MatrixXd &input, MatrixXd &output)`: specify the output behaviour of the specified system.

### `dynamic_system_controls`

This package contains implementations of dynamic systems and control functions.

Currently, the following dynamic systems are available:

- [x] `DynamicSystems::Control::IntegratorSystem`: implements a matricial integrator.
- [x] `DynamicSystems::Control::LTISystem`: implements a LTI MIMO system.

It offer the following functions:

- [x] `DynamicSystems::Control::distretization_zoh`: compute state space ZOH discretization.

---

## License

This work is licensed under the GNU General Public License v3.0. See the [`LICENSE`](../../LICENSE) file for details.

## Copyright

Copyright (c) 2023, Intelligent Systems Lab, University of Rome Tor Vergata

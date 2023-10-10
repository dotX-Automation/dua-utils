# dynamic_systems

C++ library to implement modular dynamic systems. Requires `eigen 3.4`.

## Contents

This is a collection made of the following ROS 2 packages.

### `base`

This is the base package for all dynamic systems.

`DynamicSystems::Base::InitParams` is a struct that contains the initialization parameters of the dynamic systems. It is used to initialize the dynamic systems, and should be derived to contain the parameters of the specific dynamic systems. By default, it is empty.

`DynamicSystems::Base::SetupParams` is a struct that contains the setup parameters of the dynamic systems. It is used to change the dynamic systems' behaviour, and should be derived to contain the parameters of the specific dynamic systems. By default, it is empty.

`DynamicSystems::Base::State` is a struct that contains the state of the dynamic systems. It is used to retain the whole state of the dynamic systems, and should be derived to contain the internal variables of the specific dynamic systems. By default, it is empty.

`DynamicSystems::Base::System` is an virtual class that follows the requirements of the `pluginlib` library, so it can only be used to derive new dynamic systems.

The specialization of the structures `InitParams`, `SetupParams`, and `State` shall provide deep copy functionality throught `copy` and `clone` methods.

The virtual class `System` offers the following public methods:

- `void DynamicSystems::Base::System::init(std::shared_ptr<InitParams> initParams)`: initializes the dynamic system with the given parameters.
- `void DynamicSystems::Base::System::setup(std::shared_ptr<SetupParams> setupParams)`: change the dynamic system behaviour with the given parameters.
- `bool DynamicSystems::Base::System::fini()`: finalizes the filter, freeing any allocated memory, etc..
- `bool DynamicSystems::Base::System::initialized()`: whether the system is initialized..
- `void DynamicSystems::Base::System::dirty()`: whether the output of the system must be updated..
- `bool DynamicSystems::Base::System::reset(std::shared_ptr<State> state = nullptr)`: resets the filter to last provided resetting state.
- `bool DynamicSystems::Base::System::input(MatrixXd in)`: insert input inside the system.
- `MatrixXd DynamicSystems::Base::System::output()`: get output from the system. If dirty, update the output of the system before return.
- `void DynamicSystems::Base::System::step()`: update system's internal state.
- `void DynamicSystems::Base::System::update()`: update the output of the system, if dirty.
- `MatrixXd DynamicSystems::Base::System::evolve(MatrixXd in)`: execute in order `input`, `output` and `step`.

This class also offers the following protected interface (to be specialized by derived dynamic systems):

- `void DynamicSystems::Base::System::init_parse(const InitParams& initParams)`: parse the given parameters and initialize the system.
- `void DynamicSystems::Base::System::setup_parse(const SetupParams& setupParams)`: parse the given parameters and set system's internal parameters.
- `void DynamicSystems::Base::System::setup_default()`: set default values for system's internal parameters configuration.
- `void DynamicSystems::Base::System::deinit()`: release internal structures and allocated memory used by the derived system.
- `void DynamicSystems::Base::System::state_validator(State &state)`: validate and eventually change system's internal status at every change.
- `void DynamicSystems::Base::System::input_validator(const State &state, MatrixXd &input)`: validate and eventually change the passed input eventually considering the internal state.
- `void DynamicSystems::Base::System::dynamic_map(const State &state, const MatrixXd &input, State &next)`: specify the dynamic behaviour of the specified system.
- `void DynamicSystems::Base::System::output_map(const State &state, const MatrixXd &input, MatrixXd &output)`: specify the output behaviour of the specified system.

### `control`

This package contains implementations of dynamic systems and control functions.

Currently, the following dynamic systems are available:

- [x] `DynamicSystems::Control::IntegratorSystem`: implements a matricial integrator.
- [x] `DynamicSystems::Control::LTISystem`: implements a LTI MIMO system.
- [x] `DynamicSystems::Control::PIDSystem`: implements a PID controller system.

It offer the following functions:

- [x] `DynamicSystems::Control::common_lti`: produces transfer function for zero, first and second order common linear systems.
- [x] `DynamicSystems::Control::butterworth`: computes butterworth filter transfer function from desired cutting frequencies.
- [x] `DynamicSystems::Control::realization`: computes state space realization from laplace transfer function.
- [x] `DynamicSystems::Control::distretization_zoh`: computes state space ZOH discretization.

### `filters`

This package contains implementations of filters.

Currently, the following dynamic systems are available:

- [x] `DynamicSystems::Filters::JumpFilter`: implements a jump filter to limit signal's slope.

---

## License

This work is licensed under the GNU General Public License v3.0. See the [`LICENSE`](../../LICENSE) file for details.

## Copyright

Copyright (c) 2023, Intelligent Systems Lab, University of Rome Tor Vergata

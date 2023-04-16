# filters

C++ library to implement dynamic, discrete-time scalar filters as ROS 2 plugins. Requires `pluginlib`.

## Contents

This is a collection made of the following ROS 2 packages.

### `filters_base`

This is the base package for all filters.

`Filters::FilterParams` is a struct that contains the parameters of the filter. It is used to initialize the filter, and should be derived to contain the parameters of the specific filter. By default, it contains the sampling time.

`Filters::Filter` is an abstract class that follows the requirements of the `pluginlib` library, so it can only be used to derive new filters.

The following interface is provided and must be followed in new implementations:

- `void Filters::Filter::init(std::shared_ptr<FilterParams> params)`: initializes the filter with the given parameters, taken as a shared pointer since you would initialize them as such using `pluginlib`'s class loaders. **This is necessary since `pluginlib` does not support constructors with parameters, nor copy or move semantics.**
- `void Filters::Filter::update(double sample)`: updates the filter with the given new sample, but **does not return the output**.
- `void Filters::Filter::reset()`: resets the filter to its initial state.
- `double Filters::Filter::get_sample()`: returns the last filtered sample, *i.e.*, the last valid output of the filter, or `NAN` if the filter has not been initialized yet, has been reset, its buffers are not full, etc..
- `void Filters::Filter::fini()`: finalizes the filter, freeing any allocated memory, etc..
- `void Filters::Filter::~Filter()`: destructor.

### `filters_plugins`

This package contains implementations of filters as plugins.

Currently, the following filters are available:

- [x] `MedianFilter`: implements a median filter, based on [Hoare's Quickselect algorithm](https://en.wikipedia.org/wiki/Quickselect).

---

## License

This work is licensed under the GNU General Public License v3.0. See the [`LICENSE`](../../LICENSE) file for details.

## Copyright

Copyright (c) 2023, Intelligent Systems Lab, University of Rome Tor Vergata

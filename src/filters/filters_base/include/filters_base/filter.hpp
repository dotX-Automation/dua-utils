/**
 * Filters library base class.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * April 16, 2023
 */

#ifndef FILTERS_BASE__FILTER_HPP_
#define FILTERS_BASE__FILTER_HPP_

#include <cmath>
#include <memory>

namespace Filters
{

/**
 * Holds parameters for a filter. Made to be derived from.
 */
struct FilterParams
{
  virtual ~FilterParams() = default;

  double sampling_time = NAN;
};

/**
 * Base class for filters.
 */
class Filter
{
public:
  virtual void init(std::shared_ptr<FilterParams> params) = 0;
  virtual void fini() = 0;
  virtual void update(double sample) = 0;
  virtual void reset() = 0;
  virtual double get_sample() = 0;
  virtual ~Filter() {}

protected:
  Filter() {}

  double filtered_sample_ = NAN;
  double sampling_time_ = NAN;
};

}

#endif // FILTERS_BASE__FILTER_HPP_

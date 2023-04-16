/**
 * Median filter.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * April 16, 2023
 */

#ifndef FILTERS_PLUGINS__MEDIAN_FILTER_HPP_
#define FILTERS_PLUGINS__MEDIAN_FILTER_HPP_

#include <memory>

#include <filters_base/filter.hpp>
#include <filters_base/visibility_control.h>

namespace Filters
{

/**
 * Holds parameters for a median filter.
 */
struct FILTERS_PUBLIC MedianFilterParams : public FilterParams
{
  unsigned int buffer_size = 0;
};

/**
 * Median filter.
 */
class FILTERS_PUBLIC MedianFilter : public Filter
{
public:
  void init(std::shared_ptr<FilterParams> params) override;
  void fini() override;
  void update(double sample) override;
  void reset() override;
  double get_sample() override;
  ~MedianFilter() override;

private:
  /* Buffer size */
  unsigned int buffer_size_ = 0;

  /* Samples buffer */
  double * buffer_ = nullptr;
  double * m_tmp_ = nullptr;

  /* Internal computation routines */
  void FILTERS_LOCAL add_to_buffer(double sample);
  void FILTERS_LOCAL calc_median();
  unsigned int FILTERS_LOCAL partition(unsigned int l, unsigned int r, unsigned int p);
  double FILTERS_LOCAL select(unsigned int l, unsigned int r, unsigned int k);
};

} // namespace Filters

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(Filters::MedianFilterParams, Filters::FilterParams)
PLUGINLIB_EXPORT_CLASS(Filters::MedianFilter, Filters::Filter)

#endif // FILTERS_PLUGINS__MEDIAN_FILTER_HPP_

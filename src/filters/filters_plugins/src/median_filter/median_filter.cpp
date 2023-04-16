/**
 * Median filter implementation.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * April 16, 2023
 */

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>

#include <median_filter/median_filter.hpp>

namespace Filters
{

/**
 * @brief Initializes the filter.
 *
 * @param params Filter parameters will be casted to MedianFilterParams.
 *
 * @throws InvalidArgument if parameters are not valid.
 */
void MedianFilter::init(std::shared_ptr<FilterParams> params)
{
  // Cast and check parameters
  auto median_params = std::dynamic_pointer_cast<MedianFilterParams>(params);
  if (!median_params) {
    throw std::invalid_argument("Invalid parameters for median filter.");
  }
  if (median_params->buffer_size < 1) {
    throw std::invalid_argument("Invalid buffer size for median filter.");
  }

  // Initialize filter and buffers
  sampling_time_ = median_params->sampling_time;
  buffer_size_ = median_params->buffer_size;
  if (buffer_ != nullptr) {
    delete[] buffer_;
  }
  if (m_tmp_ != nullptr) {
    delete[] m_tmp_;
  }
  buffer_ = new double[buffer_size_];
  m_tmp_ = new double[buffer_size_];
  reset();
}

/**
 * @brief Finalizes the filter.
 */
void MedianFilter::fini()
{
  delete[] buffer_;
  delete[] m_tmp_;
  m_tmp_ = nullptr;
}

/**
 * @brief Destroys the filter.
 */
MedianFilter::~MedianFilter()
{
  if (m_tmp_) {
    fini();
  }
}

/**
 * @brief Resets the filter.
 */
void MedianFilter::reset()
{
  // Reset all buffers
  filtered_sample_ = NAN;
  std::fill(buffer_, buffer_ + buffer_size_, NAN);
  std::fill(m_tmp_, m_tmp_ + buffer_size_, NAN);
}

/**
 * @brief Returns the current filtered sample.
 *
 * @return Current filtered sample, or NAN if the filter is not initialized or ready.
 */
double MedianFilter::get_sample()
{
  return filtered_sample_;
}

/**
 * @brief Updates the filter with a new sample.
 *
 * @param sample New sample.
 */
void MedianFilter::update(double sample)
{
  add_to_buffer(sample);
  if (!std::isnan(buffer_[buffer_size_ - 1])) {
    calc_median();
  }
}

/**
 * @brief Adds a new sample to the buffer.
 *
 * @param sample New sample.
 */
void MedianFilter::add_to_buffer(double sample)
{
  // Shift buffer
  std::copy(buffer_ + 1, buffer_ + buffer_size_, buffer_);

  // Add new sample
  buffer_[buffer_size_ - 1] = sample;
}

/**
 * @brief Calculates the median of the buffer.
 */
void MedianFilter::calc_median()
{
  // Copy buffer
  std::copy(buffer_, buffer_ + buffer_size_, m_tmp_);

  // Calculate median
  filtered_sample_ = select(0, buffer_size_ - 1, buffer_size_ / 2);
}

/**
 * @brief Partition function, like from quicksort.
 *
 * @param l Left array bound.
 * @param r Right array bound.
 * @param p Pivot index.
 *
 * @return Pivot index.
 */
unsigned int MedianFilter::partition(unsigned int l, unsigned int r, unsigned int p)
{
  // Swap pivot to the end
  std::swap(m_tmp_[p], m_tmp_[r]);

  // Partition
  unsigned int i = l;
  for (unsigned int j = l; j < r; j++) {
    if (m_tmp_[j] < m_tmp_[r]) {
      std::swap(m_tmp_[i], m_tmp_[j]);
      i++;
    }
  }

  // Swap pivot to its final position
  std::swap(m_tmp_[i], m_tmp_[r]);

  // Return pivot index
  return i;
}

/**
 * @brief Hoare's quickselect.
 *
 * @param l Left array bound.
 * @param r Right array bound.
 * @param k Index of the value to be returned.
 *
 * @return Value at index k.
 */
double MedianFilter::select(unsigned int l, unsigned int r, unsigned int k)
{
  // Check if we're done
  if (l == r) {
    return m_tmp_[l];
  }

  // Partition
  unsigned int p = partition(l, r, (l + r) / 2);

  // Check if we're done
  if (k == p) {
    return m_tmp_[k];
  } else if (k < p) {
    return select(l, p - 1, k);
  } else {
    return select(p + 1, r, k);
  }
}

} // namespace Filters

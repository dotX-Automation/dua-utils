/**
 * ROS 2 parameter API wrapper.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Lorenzo Bianchi <lnz.bnc@gmail.com>
 * Intelligent Systems Lab <isl.torgetgata@gmail.com>
 *
 * April 28, 2023
 */

#ifndef PARAMS_MANAGER__PARAMSMANAGER_HPP_
#define PARAMS_MANAGER__PARAMSMANAGER_HPP_

#include "visibility_control.h"

#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

/* Function used to define a custom parameter validation procedure. */
using Validator = std::function<bool (const rcl_interfaces::msg::Parameter &)>;

namespace ParamsManager
{

/* Keeps this library coherent with the ParameterType ROS message. */
enum PARAMS_MANAGER_LOCAL PType
{
  PARAMETER_NOT_SET = rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET,
  PARAMETER_BOOL = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL,
  PARAMETER_INTEGER = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER,
  PARAMETER_DOUBLE = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
  PARAMETER_STRING = rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
  PARAMETER_BYTE_ARRAY = rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY,
  PARAMETER_BOOL_ARRAY = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY,
  PARAMETER_INTEGER_ARRAY = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY,
  PARAMETER_DOUBLE_ARRAY = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY,
  PARAMETER_STRING_ARRAY = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY
};

/**
 * Stores parameter metadata required in this module.
 */
struct PARAMS_MANAGER_LOCAL ParamData
{
  std::string name_;
  PType type_;
  Validator validator_;
};

/**
 * Simplifies interactions with the ROS 2 parameter API.
 */
class PARAMS_MANAGER_PUBLIC PManager
{
public:
  /* Constructors. */
  PManager(rclcpp::Node * node);

  /* Destructor. */
  virtual ~PManager();

  /* Parameter declaration wrapper functions. */
  void declare_bool_parameter(
    std::string && name,
    bool default_val,
    std::string && desc, std::string && constraints, bool read_only,
    Validator && validator);
  void declare_bool_array_parameter(
    std::string && name,
    std::vector<bool> && default_val,
    std::string && desc, std::string && constraints, bool read_only,
    Validator && validator);
  void declare_int_parameter(
    std::string && name,
    int64_t default_val, int64_t from, int64_t to, int64_t step,
    std::string && desc, std::string && constraints, bool read_only,
    Validator && validator);
  void declare_int_array_parameter(
    std::string && name,
    std::vector<int64_t> && default_val, int64_t from, int64_t to, int64_t step,
    std::string && desc, std::string && constraints, bool read_only,
    Validator && validator);
  void declare_double_parameter(
    std::string && name,
    double default_val, double from, double to, double step,
    std::string && desc, std::string && constraints, bool read_only,
    Validator && validator);
  void declare_double_array_parameter(
    std::string && name,
    std::vector<double> && default_val, double from, double to, double step,
    std::string && desc, std::string && constraints, bool read_only,
    Validator && validator);
  void declare_string_parameter(
    std::string && name,
    std::string && default_val,
    std::string && desc, std::string && constraints, bool read_only,
    Validator && validator);
  void declare_string_array_parameter(
    std::string && name,
    std::vector<std::string> && default_val,
    std::string && desc, std::string && constraints, bool read_only,
    Validator && validator);
  void declare_byte_array_parameter(
    std::string && name,
    std::vector<uint8_t> && default_val,
    std::string && desc, std::string && constraints, bool read_only,
    Validator && validator);

  typedef std::shared_ptr<PManager> SharedPtr;

private:
  /* Reference to node using this object. */
  rclcpp::Node * node_ = nullptr;

  /* Set of parameters being managed by this object. */
  std::set<std::pair<std::string, ParamData>> params_set_;

  /* Mutex used to protect the parameters set. */
  std::mutex params_set_lock_;

  /* Parameters callback data. */
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr params_clbk_handle_ = nullptr;
  rcl_interfaces::msg::SetParametersResult PARAMS_MANAGER_LOCAL on_set_parameters_callback_(
    const std::vector<rclcpp::Parameter> & params);

  /* Internal routines */
  void PARAMS_MANAGER_LOCAL add_to_set_(
    const std::string & name,
    PType type,
    const Validator & validator);
  std::shared_ptr<ParamData> PARAMS_MANAGER_LOCAL get_param_data_(const std::string & name);
};

} // namespace ParamsManager

#endif // PARAMS_MANAGER__PARAMSMANAGER_HPP_

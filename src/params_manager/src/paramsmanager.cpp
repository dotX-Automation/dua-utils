/**
 * ROS 2 parameter API wrapper implementation.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Lorenzo Bianchi <lnz.bnc@gmail.com>
 * Intelligent Systems Lab <isl.torgetgata@gmail.com>
 *
 * April 28, 2023
 */

#include <params_manager/params_manager.hpp>

namespace ParamsManager
{

/**
 * Constructor.
 *
 * @param node Pointer to the node to which this manager is attached.
 * @param verbose Activates event logging.
 *
 * @throws InvalidArgument if the node pointer is null.
 * @throws RuntimeError if callback setting fails.
 */
PManager::PManager(rclcpp::Node * node, bool verbose)
: verbose_(verbose)
{
  // Attach to node
  if (!node) {
    throw std::invalid_argument("PManager::PManager: node pointer cannot be null");
  }
  node_ = node;

  // Set callback
  params_clbk_handle_ = node_->add_on_set_parameters_callback(
    std::bind(
      &PManager::on_set_parameters_callback_,
      this,
      std::placeholders::_1));
  if (!params_clbk_handle_) {
    throw std::runtime_error("PManager::PManager: callback setting failed");
  }
}

/**
 * Destructor.
 */
PManager::~PManager()
{
  std::scoped_lock<std::mutex> lock(params_set_lock_);
  node_->remove_on_set_parameters_callback(params_clbk_handle_.get());
  params_set_.clear();
}

/**
 * @brief Returns the parameter metadata for the given paramenter name, if present.
 *
 * @param name Parameter name.
 *
 * @return Pointer to the parameter metadata, or null if not present.
 */
std::shared_ptr<ParamData> PManager::get_param_data_(const std::string & name)
{
  std::scoped_lock<std::mutex> lock(params_set_lock_);

  auto it = params_set_.find(std::pair<std::string, ParamData>(name, ParamData{}));
  if (it != params_set_.end()) {
    return std::make_shared<ParamData>(ParamData(it->second));
  }
  return nullptr;
}

/**
 * @brief Adds a parameter to the parameter set.
 *
 * @param name Parameter name.
 * @param type Parameter type.
 * @param validator External parameter validation routine.
 */
void PManager::add_to_set_(
  const std::string & name,
  PType type,
  const Validator & validator)
{
  std::scoped_lock<std::mutex> lock(params_set_lock_);

  ParamData data(name, type, validator);
  params_set_.insert(std::pair<std::string, ParamData>(name, data));
}

/**
 * @brief Logs a parameter update event.
 *
 * @param p Parameter to be logged.
 */
void PManager::log_update_(const rclcpp::Parameter & p)
{
  std::string msg = "PManager::log_update_: '" + p.get_name() + "': ";
  switch (static_cast<PType>(p.get_type())) {
    case PType::PARAMETER_BOOL:
      msg += p.as_bool() ? "true" : "false";
      break;
    case PType::PARAMETER_INTEGER:
      msg += std::to_string(p.as_int());
      break;
    case PType::PARAMETER_DOUBLE:
      msg += std::to_string(p.as_double());
      break;
    case PType::PARAMETER_STRING:
      msg += "\'";
      msg += p.as_string();
      msg += "\'";
      break;
    case PType::PARAMETER_BYTE_ARRAY:
      char first[9], last[9];
      for (unsigned long int i = 0, j = p.as_byte_array().size() - 8; i < 8; i++, j++) {
        first[i] = p.as_byte_array()[i];
        last[i] = p.as_byte_array()[j];
      }
      first[8] = '\0';
      last[8] = '\0';
      msg += "[ ";
      msg += std::string(first);
      msg += " ... ";
      msg += std::string(last);
      msg += " ]";
      break;
    case PType::PARAMETER_BOOL_ARRAY:
      msg += "[ ";
      for (bool b : p.as_bool_array()) {
        msg += b ? "true" : "false";
        msg += ", ";
      }
      msg += "]";
      break;
    case PType::PARAMETER_INTEGER_ARRAY:
      msg += "[ ";
      for (int i : p.as_integer_array()) {
        msg += std::to_string(i);
        msg += ", ";
      }
      msg += "]";
      break;
    case PType::PARAMETER_DOUBLE_ARRAY:
      msg += "[ ";
      for (double d : p.as_double_array()) {
        msg += std::to_string(d);
        msg += ", ";
      }
      msg += "]";
      break;
    case PType::PARAMETER_STRING_ARRAY:
      msg += "[ ";
      for (std::string s : p.as_string_array()) {
        msg += "\'";
        msg += s;
        msg += "\'";
        msg += ", ";
      }
      msg += "]";
      break;
    default:
      msg += "unknown type";
      break;
  }
  RCLCPP_INFO(node_->get_logger(), msg.c_str());
}

/**
 * @brief Callback routine for parameter setting.
 *
 * @param params Parameters to be set.
 *
 * @return SetParametersResult object to be forwarded to the ROS 2 parameters subsystem.
 */
rcl_interfaces::msg::SetParametersResult PManager::on_set_parameters_callback_(
  const std::vector<rclcpp::Parameter> & params)
{
  // Initialize result object
  rcl_interfaces::msg::SetParametersResult res{};
  res.set__successful(true);
  res.set__reason("");

  // Check and update parameters
  for (const rclcpp::Parameter & p : params) {
    // Check if the parameter is declared and get its data
    std::shared_ptr<ParamData> data = get_param_data_(p.get_name());
    if (!data) {
      RCLCPP_INFO(
        node_->get_logger(),
        "PManager::on_set_parameters_callback_: parameter '%s' unknown to this manager",
        p.get_name().c_str());
      continue;
    }

    // Check type
    if (static_cast<PType>(p.get_type()) != data->type_) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "PManager::on_set_parameters_callback_: parameter '%s' type mismatch",
        p.get_name().c_str());
      res.set__successful(false);
      res.set__reason("Parameter '" + p.get_name() + "' type mismatch");
      return res;
    }

    // Run validator, if present
    if (data->validator_ && !data->validator_(p)) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "PManager::on_set_parameters_callback_: parameter '%s' validation failed",
        p.get_name().c_str());
      res.set__successful(false);
      res.set__reason("Parameter '" + p.get_name() + "' validation failed");
      return res;
    }

    // Log update
    if (verbose_) {
      log_update_(p);
    }
  }

  return res;
}

/**
 * @brief Routine to declare a boolean node parameter.
 *
 * @param name Parameter name.
 * @param default_val Default value.
 * @param desc Parameter description.
 * @param constraints Additional value constraints.
 * @param read_only Read-only internal flag.
 * @param validator Parameter validation routine.
 *
 * @throws InvalidArgument if the parameter is already declared.
 */
void PManager::declare_bool_parameter(
  std::string && name,
  bool default_val,
  std::string && desc, std::string && constraints, bool read_only,
  Validator && validator)
{
  // Check if the parameter is not already present
  if (get_param_data_(name) != nullptr) {
    throw std::invalid_argument("PManager::declare_bool_parameter: parameter already declared");
  }

  // Add parameter to the set
  add_to_set_(name, PType::PARAMETER_BOOL, validator);

  // Declare parameter
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.set__name(name);
  descriptor.set__type(static_cast<uint8_t>(PType::PARAMETER_BOOL));
  descriptor.set__description(desc);
  descriptor.set__additional_constraints(constraints);
  descriptor.set__read_only(read_only);
  descriptor.set__dynamic_typing(false);
  node_->declare_parameter(name, default_val, descriptor);
}

/**
 * @brief Routine to declare a boolean array node parameter.
 *
 * @param name Parameter name.
 * @param default_val Default value.
 * @param desc Parameter description.
 * @param constraints Additional value constraints.
 * @param read_only Read-only internal flag.
 * @param validator Parameter validation routine.
 *
 * @throws InvalidArgument if the parameter is already declared.
 */
void PManager::declare_bool_array_parameter(
  std::string && name,
  std::vector<bool> && default_val,
  std::string && desc, std::string && constraints, bool read_only,
  Validator && validator)
{
  // Check if the parameter is not already present
  if (get_param_data_(name) != nullptr) {
    throw std::invalid_argument("PManager::declare_bool_array_parameter: parameter already declared");
  }

  // Add parameter to the set
  add_to_set_(name, PType::PARAMETER_BOOL_ARRAY, validator);

  // Declare parameter
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.set__name(name);
  descriptor.set__type(static_cast<uint8_t>(PType::PARAMETER_BOOL_ARRAY));
  descriptor.set__description(desc);
  descriptor.set__additional_constraints(constraints);
  descriptor.set__read_only(read_only);
  descriptor.set__dynamic_typing(false);
  node_->declare_parameter(name, default_val, descriptor);
}

/**
 * @brief Routine to declare an integer node parameter.
 *
 * @param name Parameter name.
 * @param default_val Default value.
 * @param from Integer range initial value.
 * @param to Integer range final value.
 * @param step Integer range step.
 * @param desc Parameter description.
 * @param constraints Additional value constraints.
 * @param read_only Read-only internal flag.
 * @param validator Paramter validation routine.
 *
 * @throws InvalidArgument if the parameter is already declared.
 */
void PManager::declare_integer_parameter(
  std::string && name,
  int64_t default_val, int64_t from, int64_t to, int64_t step,
  std::string && desc, std::string && constraints, bool read_only,
  Validator && validator)
{
  // Check if the parameter is not already present
  if (get_param_data_(name) != nullptr) {
    throw std::invalid_argument("PManager::declare_int_parameter: parameter already declared");
  }

  // Add parameter to the set
  add_to_set_(name, PType::PARAMETER_INTEGER, validator);

  // Declare parameter
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  rcl_interfaces::msg::IntegerRange param_range{};
  param_range.set__from_value(from);
  param_range.set__to_value(to);
  param_range.set__step(step);
  descriptor.set__name(name);
  descriptor.set__type(static_cast<uint8_t>(PType::PARAMETER_INTEGER));
  descriptor.set__description(desc);
  descriptor.set__additional_constraints(constraints);
  descriptor.set__read_only(read_only);
  descriptor.set__dynamic_typing(false);
  descriptor.set__integer_range({param_range});
  node_->declare_parameter(name, default_val, descriptor);
}

/**
 * @brief Routine to declare an integer array node parameter.
 *
 * @param name Parameter name.
 * @param default_val Default value.
 * @param from Integer range initial value.
 * @param to Integer range final value.
 * @param step Integer range step.
 * @param desc Parameter description.
 * @param constraints Additional value constraints.
 * @param read_only Read-only internal flag.
 * @param validator Paramter validation routine.
 *
 * @throws InvalidArgument if the parameter is already declared.
 */
void PManager::declare_integer_array_parameter(
  std::string && name,
  std::vector<int64_t> && default_val, int64_t from, int64_t to, int64_t step,
  std::string && desc, std::string && constraints, bool read_only,
  Validator && validator)
{
  // Check if the parameter is not already present
  if (get_param_data_(name) != nullptr) {
    throw std::invalid_argument("PManager::declare_int_array_parameter: parameter already declared");
  }

  // Add parameter to the set
  add_to_set_(name, PType::PARAMETER_INTEGER_ARRAY, validator);

  // Declare parameter
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  rcl_interfaces::msg::IntegerRange param_range{};
  param_range.set__from_value(from);
  param_range.set__to_value(to);
  param_range.set__step(step);
  descriptor.set__name(name);
  descriptor.set__type(static_cast<uint8_t>(PType::PARAMETER_INTEGER_ARRAY));
  descriptor.set__description(desc);
  descriptor.set__additional_constraints(constraints);
  descriptor.set__read_only(read_only);
  descriptor.set__dynamic_typing(false);
  descriptor.set__integer_range({param_range});
  node_->declare_parameter(name, default_val, descriptor);
}

/**
 * @brief Routine to declare a 64-bit floating point node parameter.
 *
 * @param name Parameter name.
 * @param default_val Default value.
 * @param from Floating point range initial value.
 * @param to Floating point range final value.
 * @param step Floating point range step.
 * @param desc Parameter description.
 * @param constraints Additional value constraints.
 * @param read_only Read-only internal flag.
 * @param validator Parameter validation routine.
 *
 * @throws InvalidArgument if the parameter is already declared.
 */
void PManager::declare_double_parameter(
  std::string && name,
  double default_val, double from, double to, double step,
  std::string && desc, std::string && constraints, bool read_only,
  Validator && validator)
{
  // Check if the parameter is not already present
  if (get_param_data_(name) != nullptr) {
    throw std::invalid_argument("PManager::declare_double_parameter: parameter already declared");
  }

  // Add parameter to the set
  add_to_set_(name, PType::PARAMETER_DOUBLE, validator);

  // Declare parameter
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  rcl_interfaces::msg::FloatingPointRange param_range{};
  param_range.set__from_value(from);
  param_range.set__to_value(to);
  param_range.set__step(step);
  descriptor.set__name(name);
  descriptor.set__type(static_cast<uint8_t>(PType::PARAMETER_DOUBLE));
  descriptor.set__description(desc);
  descriptor.set__additional_constraints(constraints);
  descriptor.set__read_only(read_only);
  descriptor.set__dynamic_typing(false);
  descriptor.set__floating_point_range({param_range});
  node_->declare_parameter(name, default_val, descriptor);
}

/**
 * @brief Routine to declare a 64-bit floating point array node parameter.
 *
 * @param name Parameter name.
 * @param default_val Default value.
 * @param from Floating point range initial value.
 * @param to Floating point range final value.
 * @param step Floating point range step.
 * @param desc Parameter description.
 * @param constraints Additional value constraints.
 * @param read_only Read-only internal flag.
 * @param validator Parameter validation routine.
 *
 * @throws InvalidArgument if the parameter is already declared.
 */
void PManager::declare_double_array_parameter(
  std::string && name,
  std::vector<double> && default_val, double from, double to, double step,
  std::string && desc, std::string && constraints, bool read_only,
  Validator && validator)
{
  // Check if the parameter is not already present
  if (get_param_data_(name) != nullptr) {
    throw std::invalid_argument(
            "PManager::declare_double_array_parameter: parameter already declared");
  }

  // Add parameter to the set
  add_to_set_(name, PType::PARAMETER_DOUBLE_ARRAY, validator);

  // Declare parameter
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  rcl_interfaces::msg::FloatingPointRange param_range{};
  param_range.set__from_value(from);
  param_range.set__to_value(to);
  param_range.set__step(step);
  descriptor.set__name(name);
  descriptor.set__type(static_cast<uint8_t>(PType::PARAMETER_DOUBLE_ARRAY));
  descriptor.set__description(desc);
  descriptor.set__additional_constraints(constraints);
  descriptor.set__read_only(read_only);
  descriptor.set__dynamic_typing(false);
  descriptor.set__floating_point_range({param_range});
  node_->declare_parameter(name, default_val, descriptor);
}

/**
 * @brief Routine to declare a string node parameter.
 *
 * @param name Parameter name.
 * @param default_val Default value.
 * @param desc Parameter description.
 * @param constraints Additional value constraints.
 * @param read_only Read-only internal flag.
 * @param validator Parameter validation routine.
 *
 * @throws InvalidArgument if the parameter is already declared.
 */
void PManager::declare_string_parameter(
  std::string && name,
  std::string && default_val,
  std::string && desc, std::string && constraints, bool read_only,
  Validator && validator)
{
  // Check if the parameter is not already present
  if (get_param_data_(name) != nullptr) {
    throw std::invalid_argument("PManager::declare_string_parameter: parameter already declared");
  }

  // Add parameter to the set
  add_to_set_(name, PType::PARAMETER_STRING, validator);

  // Declare parameter
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.set__name(name);
  descriptor.set__type(static_cast<uint8_t>(PType::PARAMETER_STRING));
  descriptor.set__description(desc);
  descriptor.set__additional_constraints(constraints);
  descriptor.set__read_only(read_only);
  descriptor.set__dynamic_typing(false);
  node_->declare_parameter(name, default_val, descriptor);
}

/**
 * @brief Routine to declare a string array node parameter.
 *
 * @param name Parameter name.
 * @param default_val Default value.
 * @param desc Parameter description.
 * @param constraints Additional value constraints.
 * @param read_only Read-only internal flag.
 * @param validator Parameter validation routine.
 *
 * @throws InvalidArgument if the parameter is already declared.
 */
void PManager::declare_string_array_parameter(
  std::string && name,
  std::vector<std::string> && default_val,
  std::string && desc, std::string && constraints, bool read_only,
  Validator && validator)
{
  // Check if the parameter is not already present
  if (get_param_data_(name) != nullptr) {
    throw std::invalid_argument(
            "PManager::declare_string_array_parameter: parameter already declared");
  }

  // Add parameter to the set
  add_to_set_(name, PType::PARAMETER_STRING_ARRAY, validator);

  // Declare parameter
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.set__name(name);
  descriptor.set__type(static_cast<uint8_t>(PType::PARAMETER_STRING_ARRAY));
  descriptor.set__description(desc);
  descriptor.set__additional_constraints(constraints);
  descriptor.set__read_only(read_only);
  descriptor.set__dynamic_typing(false);
  node_->declare_parameter(name, default_val, descriptor);
}

/**
 * @brief Routine to declare a byte array node parameter.
 *
 * @param name Parameter name.
 * @param default_val Default value.
 * @param desc Parameter description.
 * @param constraints Additional value constraints.
 * @param read_only Read-only internal flag.
 * @param validator Parameter validation routine.
 *
 * @throws InvalidArgument if the parameter is already declared.
 */
void PManager::declare_byte_array_parameter(
  std::string && name,
  std::vector<uint8_t> && default_val,
  std::string && desc, std::string && constraints, bool read_only,
  Validator && validator)
{
  // Check if the parameter is not already present
  if (get_param_data_(name) != nullptr) {
    throw std::invalid_argument("PManager::declare_byte_array_parameter: parameter already declared");
  }

  // Add parameter to the set
  add_to_set_(name, PType::PARAMETER_BYTE_ARRAY, validator);

  // Declare parameter
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.set__name(name);
  descriptor.set__type(static_cast<uint8_t>(PType::PARAMETER_BYTE_ARRAY));
  descriptor.set__description(desc);
  descriptor.set__additional_constraints(constraints);
  descriptor.set__read_only(read_only);
  descriptor.set__dynamic_typing(false);
  node_->declare_parameter(name, default_val, descriptor);
}

} // namespace ParamsManager

// auto-generated DO NOT EDIT

#pragma once

#include <algorithm>
#include <array>
#include <functional>
#include <limits>
#include <mutex>
#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/logger.hpp>
#include <set>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <fmt/core.h>
#include <fmt/format.h>
#include <fmt/ranges.h>

#include <parameter_traits/parameter_traits.hpp>

#include <rsl/static_string.hpp>
#include <rsl/static_vector.hpp>
#include <rsl/parameter_validators.hpp>



namespace steering_controllers_library {

// Use validators from RSL
using rsl::unique;
using rsl::subset_of;
using rsl::fixed_size;
using rsl::size_gt;
using rsl::size_lt;
using rsl::not_empty;
using rsl::element_bounds;
using rsl::lower_element_bounds;
using rsl::upper_element_bounds;
using rsl::bounds;
using rsl::lt;
using rsl::gt;
using rsl::lt_eq;
using rsl::gt_eq;
using rsl::one_of;
using rsl::to_parameter_result_msg;

// temporarily needed for backwards compatibility for custom validators
using namespace parameter_traits;

template <typename T>
[[nodiscard]] auto to_parameter_value(T value) {
    return rclcpp::ParameterValue(value);
}

template <size_t capacity>
[[nodiscard]] auto to_parameter_value(rsl::StaticString<capacity> const& value) {
    return rclcpp::ParameterValue(rsl::to_string(value));
}

template <typename T, size_t capacity>
[[nodiscard]] auto to_parameter_value(rsl::StaticVector<T, capacity> const& value) {
    return rclcpp::ParameterValue(rsl::to_vector(value));
}
    struct Params {
        double reference_timeout = 1.0;
        bool front_steering = true;
        std::vector<std::string> rear_wheels_names = {};
        std::vector<std::string> traction_joints_names = {};
        std::vector<std::string> front_wheels_names = {};
        std::vector<std::string> steering_joints_names = {};
        std::vector<std::string> traction_joints_state_names = {};
        std::vector<std::string> rear_wheels_state_names = {};
        std::vector<std::string> steering_joints_state_names = {};
        std::vector<std::string> front_wheels_state_names = {};
        bool open_loop = false;
        bool reduce_wheel_speed_until_steering_reached = false;
        int64_t velocity_rolling_window_size = 10;
        std::string base_frame_id = "base_link";
        std::string odom_frame_id = "odom";
        bool enable_odom_tf = true;
        std::vector<double> twist_covariance_diagonal = {0.0, 7.0, 14.0, 21.0, 28.0, 35.0};
        std::vector<double> pose_covariance_diagonal = {0.0, 7.0, 14.0, 21.0, 28.0, 35.0};
        bool position_feedback = false;
        // for detecting if the parameter struct has been updated
        rclcpp::Time __stamp;
    };
    struct StackParams {
        double reference_timeout = 1.0;
        bool front_steering = true;
        bool open_loop = false;
        bool reduce_wheel_speed_until_steering_reached = false;
        int64_t velocity_rolling_window_size = 10;
        bool enable_odom_tf = true;
        bool position_feedback = false;
    };

  class ParamListener{
  public:
    // throws rclcpp::exceptions::InvalidParameterValueException on initialization if invalid parameter are loaded
    ParamListener(rclcpp::Node::SharedPtr node, std::string const& prefix = "")
    : ParamListener(node->get_node_parameters_interface(), node->get_logger(), prefix) {}

    ParamListener(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string const& prefix = "")
    : ParamListener(node->get_node_parameters_interface(), node->get_logger(), prefix) {}

    ParamListener(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>& parameters_interface,
                  std::string const& prefix = "")
    : ParamListener(parameters_interface, rclcpp::get_logger("steering_controllers_library"), prefix) {
      RCLCPP_DEBUG(logger_, "ParameterListener: Not using node logger, recommend using other constructors to use a node logger");
    }

    ParamListener(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>& parameters_interface,
                  rclcpp::Logger logger, std::string const& prefix = "") {
      logger_ = std::move(logger);
      prefix_ = prefix;
      if (!prefix_.empty() && prefix_.back() != '.') {
        prefix_ += ".";
      }

      parameters_interface_ = parameters_interface;
      declare_params();
      auto update_param_cb = [this](const std::vector<rclcpp::Parameter> &parameters){return this->update(parameters);};
      handle_ = parameters_interface_->add_on_set_parameters_callback(update_param_cb);
      clock_ = rclcpp::Clock();
    }

    Params get_params() const{
      std::lock_guard<std::mutex> lock(mutex_);
      return params_;
    }

    bool try_get_params(Params & params_in) const {
      if (mutex_.try_lock()) {
        if (const bool is_old = params_in.__stamp != params_.__stamp; is_old) {
          params_in = params_;
        }
        mutex_.unlock();
        return true;
      }
      return false;
    }

    bool is_old(Params const& other) const {
      std::lock_guard<std::mutex> lock(mutex_);
      return params_.__stamp != other.__stamp;
    }

    StackParams get_stack_params() {
      Params params = get_params();
      StackParams output;
      output.reference_timeout = params.reference_timeout;
      output.front_steering = params.front_steering;
      output.open_loop = params.open_loop;
      output.reduce_wheel_speed_until_steering_reached = params.reduce_wheel_speed_until_steering_reached;
      output.velocity_rolling_window_size = params.velocity_rolling_window_size;
      output.enable_odom_tf = params.enable_odom_tf;
      output.position_feedback = params.position_feedback;

      return output;
    }

    void refresh_dynamic_parameters() {
      auto updated_params = get_params();
      // TODO remove any destroyed dynamic parameters

      // declare any new dynamic parameters
      rclcpp::Parameter param;

    }

    rcl_interfaces::msg::SetParametersResult update(const std::vector<rclcpp::Parameter> &parameters) {
      auto updated_params = get_params();

      for (const auto &param: parameters) {
        if (param.get_name() == (prefix_ + "reference_timeout")) {
            updated_params.reference_timeout = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "front_steering")) {
            updated_params.front_steering = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "rear_wheels_names")) {
            if(auto validation_result = size_lt<std::string>(param, 5);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            if(auto validation_result = unique<std::string>(param);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.rear_wheels_names = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "traction_joints_names")) {
            if(auto validation_result = size_lt<std::string>(param, 5);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            if(auto validation_result = unique<std::string>(param);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.traction_joints_names = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "front_wheels_names")) {
            if(auto validation_result = size_lt<std::string>(param, 5);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            if(auto validation_result = unique<std::string>(param);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.front_wheels_names = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "steering_joints_names")) {
            if(auto validation_result = size_lt<std::string>(param, 5);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            if(auto validation_result = unique<std::string>(param);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.steering_joints_names = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "traction_joints_state_names")) {
            if(auto validation_result = size_lt<std::string>(param, 5);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            if(auto validation_result = unique<std::string>(param);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.traction_joints_state_names = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "rear_wheels_state_names")) {
            if(auto validation_result = size_lt<std::string>(param, 5);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            if(auto validation_result = unique<std::string>(param);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.rear_wheels_state_names = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "steering_joints_state_names")) {
            if(auto validation_result = size_lt<std::string>(param, 5);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            if(auto validation_result = unique<std::string>(param);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.steering_joints_state_names = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "front_wheels_state_names")) {
            if(auto validation_result = size_lt<std::string>(param, 5);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            if(auto validation_result = unique<std::string>(param);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.front_wheels_state_names = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "open_loop")) {
            updated_params.open_loop = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "reduce_wheel_speed_until_steering_reached")) {
            updated_params.reduce_wheel_speed_until_steering_reached = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "velocity_rolling_window_size")) {
            updated_params.velocity_rolling_window_size = param.as_int();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "base_frame_id")) {
            updated_params.base_frame_id = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "odom_frame_id")) {
            updated_params.odom_frame_id = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "enable_odom_tf")) {
            updated_params.enable_odom_tf = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "twist_covariance_diagonal")) {
            updated_params.twist_covariance_diagonal = param.as_double_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "pose_covariance_diagonal")) {
            updated_params.pose_covariance_diagonal = param.as_double_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "position_feedback")) {
            updated_params.position_feedback = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
      }

      updated_params.__stamp = clock_.now();
      update_internal_params(updated_params);
      return rsl::to_parameter_result_msg({});
    }

    void declare_params(){
      auto updated_params = get_params();
      // declare all parameters and give default values to non-required ones
      if (!parameters_interface_->has_parameter(prefix_ + "reference_timeout")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Timeout for controller references after which they will be reset. This is especially useful for controllers that can cause unwanted and dangerous behavior if reference is not reset, e.g., velocity controllers. If value is 0 the reference is reset after each run.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.reference_timeout);
          parameters_interface_->declare_parameter(prefix_ + "reference_timeout", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "front_steering")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "DEPRECATED: Use 'traction_joints_names' or 'steering_joints_names' instead";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.front_steering);
          parameters_interface_->declare_parameter(prefix_ + "front_steering", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "rear_wheels_names")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "DEPRECATED: Use 'traction_joints_names' or 'steering_joints_names' instead";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.rear_wheels_names);
          parameters_interface_->declare_parameter(prefix_ + "rear_wheels_names", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "traction_joints_names")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Names of traction wheel joints. For kinematic configurations with two traction joints, the expected order is: right joint, left joint.";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.traction_joints_names);
          parameters_interface_->declare_parameter(prefix_ + "traction_joints_names", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "front_wheels_names")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "DEPRECATED: Use 'traction_joints_names' or 'steering_joints_names'' instead";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.front_wheels_names);
          parameters_interface_->declare_parameter(prefix_ + "front_wheels_names", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "steering_joints_names")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Names of steering joints. For kinematic configurations with two steering joints, the expected order is: right joint, left joint. The orientation of the steering axes is expected as such: When positive steering position value is commanded, then the robot should turn in positive direction of the z-axis of the vehicle (see REP-103).";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.steering_joints_names);
          parameters_interface_->declare_parameter(prefix_ + "steering_joints_names", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "traction_joints_state_names")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "(Optional) Names of tractions joints to read states from. If not set joint names from 'traction_joints_names' will be used.";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.traction_joints_state_names);
          parameters_interface_->declare_parameter(prefix_ + "traction_joints_state_names", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "rear_wheels_state_names")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "DEPRECATED: Use 'traction_joints_state_names' or 'steering_joints_state_names' instead";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.rear_wheels_state_names);
          parameters_interface_->declare_parameter(prefix_ + "rear_wheels_state_names", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "steering_joints_state_names")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "(Optional) Names of steering joints to read states from. If not set joint names from 'steering_joints_names' will be used.";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.steering_joints_state_names);
          parameters_interface_->declare_parameter(prefix_ + "steering_joints_state_names", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "front_wheels_state_names")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "DEPRECATED: Use 'traction_joints_state_names' or 'steering_joints_state_names' instead";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.front_wheels_state_names);
          parameters_interface_->declare_parameter(prefix_ + "front_wheels_state_names", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "open_loop")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Choose if open-loop or not (feedback) is used for odometry calculation.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.open_loop);
          parameters_interface_->declare_parameter(prefix_ + "open_loop", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "reduce_wheel_speed_until_steering_reached")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Reduce wheel speed until the steering angle has been reached.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.reduce_wheel_speed_until_steering_reached);
          parameters_interface_->declare_parameter(prefix_ + "reduce_wheel_speed_until_steering_reached", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "velocity_rolling_window_size")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The number of velocity samples to average together to compute the odometry twist.linear.x and twist.angular.z velocities.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.velocity_rolling_window_size);
          parameters_interface_->declare_parameter(prefix_ + "velocity_rolling_window_size", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "base_frame_id")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Base frame_id set to value of base_frame_id.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.base_frame_id);
          parameters_interface_->declare_parameter(prefix_ + "base_frame_id", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "odom_frame_id")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Odometry frame_id set to value of odom_frame_id.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.odom_frame_id);
          parameters_interface_->declare_parameter(prefix_ + "odom_frame_id", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "enable_odom_tf")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Publishing to tf is enabled or disabled?";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.enable_odom_tf);
          parameters_interface_->declare_parameter(prefix_ + "enable_odom_tf", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "twist_covariance_diagonal")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "diagonal values of twist covariance matrix.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.twist_covariance_diagonal);
          parameters_interface_->declare_parameter(prefix_ + "twist_covariance_diagonal", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "pose_covariance_diagonal")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "diagonal values of pose covariance matrix.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.pose_covariance_diagonal);
          parameters_interface_->declare_parameter(prefix_ + "pose_covariance_diagonal", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "position_feedback")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Choice of feedback type, if position_feedback is false then HW_IF_VELOCITY is taken as interface type, if position_feedback is true then HW_IF_POSITION is taken as interface type";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.position_feedback);
          parameters_interface_->declare_parameter(prefix_ + "position_feedback", parameter, descriptor);
      }
      // get parameters and fill struct fields
      rclcpp::Parameter param;
      param = parameters_interface_->get_parameter(prefix_ + "reference_timeout");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.reference_timeout = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "front_steering");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.front_steering = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "rear_wheels_names");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = size_lt<std::string>(param, 5);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'rear_wheels_names': {}", validation_result.error()));
      }
      if(auto validation_result = unique<std::string>(param);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'rear_wheels_names': {}", validation_result.error()));
      }
      updated_params.rear_wheels_names = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "traction_joints_names");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = size_lt<std::string>(param, 5);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'traction_joints_names': {}", validation_result.error()));
      }
      if(auto validation_result = unique<std::string>(param);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'traction_joints_names': {}", validation_result.error()));
      }
      updated_params.traction_joints_names = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "front_wheels_names");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = size_lt<std::string>(param, 5);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'front_wheels_names': {}", validation_result.error()));
      }
      if(auto validation_result = unique<std::string>(param);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'front_wheels_names': {}", validation_result.error()));
      }
      updated_params.front_wheels_names = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "steering_joints_names");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = size_lt<std::string>(param, 5);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'steering_joints_names': {}", validation_result.error()));
      }
      if(auto validation_result = unique<std::string>(param);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'steering_joints_names': {}", validation_result.error()));
      }
      updated_params.steering_joints_names = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "traction_joints_state_names");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = size_lt<std::string>(param, 5);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'traction_joints_state_names': {}", validation_result.error()));
      }
      if(auto validation_result = unique<std::string>(param);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'traction_joints_state_names': {}", validation_result.error()));
      }
      updated_params.traction_joints_state_names = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "rear_wheels_state_names");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = size_lt<std::string>(param, 5);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'rear_wheels_state_names': {}", validation_result.error()));
      }
      if(auto validation_result = unique<std::string>(param);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'rear_wheels_state_names': {}", validation_result.error()));
      }
      updated_params.rear_wheels_state_names = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "steering_joints_state_names");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = size_lt<std::string>(param, 5);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'steering_joints_state_names': {}", validation_result.error()));
      }
      if(auto validation_result = unique<std::string>(param);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'steering_joints_state_names': {}", validation_result.error()));
      }
      updated_params.steering_joints_state_names = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "front_wheels_state_names");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = size_lt<std::string>(param, 5);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'front_wheels_state_names': {}", validation_result.error()));
      }
      if(auto validation_result = unique<std::string>(param);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'front_wheels_state_names': {}", validation_result.error()));
      }
      updated_params.front_wheels_state_names = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "open_loop");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.open_loop = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "reduce_wheel_speed_until_steering_reached");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.reduce_wheel_speed_until_steering_reached = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "velocity_rolling_window_size");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.velocity_rolling_window_size = param.as_int();
      param = parameters_interface_->get_parameter(prefix_ + "base_frame_id");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.base_frame_id = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "odom_frame_id");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.odom_frame_id = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "enable_odom_tf");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.enable_odom_tf = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "twist_covariance_diagonal");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.twist_covariance_diagonal = param.as_double_array();
      param = parameters_interface_->get_parameter(prefix_ + "pose_covariance_diagonal");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.pose_covariance_diagonal = param.as_double_array();
      param = parameters_interface_->get_parameter(prefix_ + "position_feedback");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.position_feedback = param.as_bool();


      updated_params.__stamp = clock_.now();
      update_internal_params(updated_params);
    }

    private:
      void update_internal_params(Params updated_params) {
        std::lock_guard<std::mutex> lock(mutex_);
        params_ = std::move(updated_params);
      }

      std::string prefix_;
      Params params_;
      rclcpp::Clock clock_;
      std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> handle_;
      std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface_;

      // rclcpp::Logger cannot be default-constructed
      // so we must provide a initialization here even though
      // every one of our constructors initializes logger_
      rclcpp::Logger logger_ = rclcpp::get_logger("steering_controllers_library");
      std::mutex mutable mutex_;
  };

} // namespace steering_controllers_library

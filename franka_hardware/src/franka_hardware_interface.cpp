// Copyright (c) 2021 Franka Emika GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <franka_hardware/franka_hardware_interface.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>

#include <franka/exception.h>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace franka_hardware {

using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;

std::vector<StateInterface> FrankaHardwareInterface::export_state_interfaces() {
  std::vector<StateInterface> state_interfaces;

  for (auto i = 0U; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_.at(i)));
    state_interfaces.emplace_back(StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_.at(i)));
    state_interfaces.emplace_back(StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_.at(i)));
  }

  const std::string cart_pos_prefix = arm_id_ + "_ee_cartesian_position";
  const std::string cart_vel_prefix = arm_id_ + "_ee_cartesian_velocity";
  for (auto i = 0; i < 16; i++){
    state_interfaces.emplace_back(StateInterface(
        cart_pos_prefix, cartesian_matrix_names[i], &hw_cartesian_positions_[i]));
    state_interfaces.emplace_back(StateInterface(
        cart_vel_prefix, cartesian_matrix_names[i], &hw_cartesian_velocities_[i]));
  }

  state_interfaces.emplace_back(StateInterface(
      arm_id_, k_robot_state_interface_name,
      reinterpret_cast<double*>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
          &hw_franka_robot_state_addr_)));
  state_interfaces.emplace_back(StateInterface(
      arm_id_, k_robot_model_interface_name,
      reinterpret_cast<double*>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
          &hw_franka_model_ptr_)));
  
  return state_interfaces;
}

std::vector<CommandInterface> FrankaHardwareInterface::export_command_interfaces() {
  std::vector<CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());
  //RCLCPP_INFO(getLogger(), "%ld", info_.joints.size());

  for (auto i = 0U; i < info_.joints.size(); i++) {
    //RCLCPP_INFO(getLogger(), "%s", info_.joints[i].name.c_str());
    command_interfaces.emplace_back(CommandInterface( // JOINT EFFORT
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_joint_effort.at(i)));
    command_interfaces.emplace_back(CommandInterface( // JOINT POSITION
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_joint_position.at(i)));
    command_interfaces.emplace_back(CommandInterface( // JOINT VELOCITY
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_joint_velocity.at(i)));
  }
  const std::string cart_pos_prefix_cmd = arm_id_ + "_ee_cartesian_position";
  const std::string cart_vel_prefix_cmd = arm_id_ + "_ee_cartesian_velocity";
  for (auto i = 0; i < 16; i++){
    command_interfaces.emplace_back(CommandInterface(
        cart_pos_prefix_cmd, cartesian_matrix_names[i], &hw_commands_cartesian_position[i]));
  }
  for (auto i = 0; i < 6; i++){
    command_interfaces.emplace_back(CommandInterface(
        cart_vel_prefix_cmd, cartesian_velocity_command_names[i], &hw_commands_cartesian_velocity[i]));
  }
  // Franka ros provides:
  // joint torque, position, velocity
  // cartesian position, velocity 
  // in total, 5 interfaces

  return command_interfaces;
}

CallbackReturn FrankaHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  robot_->initializeContinuousReading();
  hw_commands_joint_effort.fill(0);
  read(rclcpp::Time(0),
       rclcpp::Duration(0, 0));  // makes sure that the robot state is properly initialized.

  // GUIDING auto-pause supervisor: spawn a small thread + auxiliary
  // node only when controller_name was supplied (see on_init).
  if (!controller_name_.empty() && !ext_node_) {
    ext_node_ = std::make_shared<rclcpp::Node>(
        arm_id_ + "_guiding_supervisor",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    switch_cli_ = ext_node_->create_client<controller_manager_msgs::srv::SwitchController>(
        "/controller_manager/switch_controller");
    finish_watch_ = false;
    guiding_watch_thread_ = std::thread([this]() { this->guidingWatchLoop(); });
  }

  RCLCPP_INFO(getLogger(), "Started");
  return CallbackReturn::SUCCESS;
}

CallbackReturn FrankaHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(getLogger(), "trying to Stop...");
  finish_watch_ = true;
  if (guiding_watch_thread_.joinable()) {
    guiding_watch_thread_.join();
  }
  switch_cli_.reset();
  ext_node_.reset();
  robot_->stopRobot();
  RCLCPP_INFO(getLogger(), "Stopped");
  return CallbackReturn::SUCCESS;
}

void FrankaHardwareInterface::guidingWatchLoop() {
  // Wait for controller_manager service to become available (we are
  // loaded by it, but service discovery still needs a moment).
  if (!switch_cli_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_WARN(getLogger(),
                "switch_controller service did not appear within 5s — "
                "GUIDING auto-pause for controller '%s' is disabled.",
                controller_name_.c_str());
    return;
  }
  bool prev = false;
  while (!finish_watch_.load()) {
    bool cur = in_guiding_.load();
    if (cur != prev) {
      auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
      if (cur) {
        req->deactivate_controllers.push_back(controller_name_);
        RCLCPP_INFO(getLogger(),
                    "GUIDING entered: deactivating '%s'", controller_name_.c_str());
      } else {
        req->activate_controllers.push_back(controller_name_);
        RCLCPP_INFO(getLogger(),
                    "GUIDING exited: re-activating '%s' (will hold at user's pose)",
                    controller_name_.c_str());
      }
      req->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
      switch_cli_->async_send_request(req);
      prev = cur;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

hardware_interface::return_type FrankaHardwareInterface::read(const rclcpp::Time& /*time*/,
                                                              const rclcpp::Duration& /*period*/) {
  
  if (hw_franka_model_ptr_ == nullptr) {
    hw_franka_model_ptr_ = robot_->getModel();
  }
  hw_franka_robot_state_ = robot_->read();
  hw_positions_ = hw_franka_robot_state_.q;
  hw_velocities_ = hw_franka_robot_state_.dq;
  hw_efforts_ = hw_franka_robot_state_.tau_J;
  hw_cartesian_positions_ = hw_franka_robot_state_.O_T_EE;
  hw_cartesian_velocities_ = hw_franka_robot_state_.O_T_EE_d;
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FrankaHardwareInterface::write(const rclcpp::Time& /*time*/,
                                                               const rclcpp::Duration& /*period*/) {
  if (std::any_of(hw_commands_joint_effort.begin(), hw_commands_joint_effort.end(),
                  [](double c) { return !std::isfinite(c); })) {
    return hardware_interface::return_type::ERROR;
  }
  if (std::any_of(hw_commands_joint_position.begin(), hw_commands_joint_position.end(),
                  [](double c) { return !std::isfinite(c); })) {
    return hardware_interface::return_type::ERROR;
  }
  if (std::any_of(hw_commands_joint_velocity.begin(), hw_commands_joint_velocity.end(),
                  [](double c) { return !std::isfinite(c); })) {
    return hardware_interface::return_type::ERROR;
  }
  if (std::any_of(hw_commands_cartesian_position.begin(), hw_commands_cartesian_position.end(),
                  [](double c) { return !std::isfinite(c); })) {
    return hardware_interface::return_type::ERROR;
  }
  if (std::any_of(hw_commands_cartesian_velocity.begin(), hw_commands_cartesian_velocity.end(),
                  [](double c) { return !std::isfinite(c); })) {
    return hardware_interface::return_type::ERROR;
  }


  if(robot_->hasError()){
    //RCLCPP_INFO(getLogger(), "write error");
    // as soon as an error is returned, it prevents read/write from running in the control node loop.
    // Need a way to make it recover from it...
    //return hardware_interface::return_type::ERROR;
    return hardware_interface::return_type::OK;
  }

  // GUIDING auto-pause (patch (h+)): if libfranka is in kGuiding
  // (operator pressed the Guiding Button on the Pilot-Grip while
  // half-pressing the Enabling Button), zero the user-supplied torque /
  // velocity commands and pin commanded position to current actual.
  // Without this, JTC's tracking PID fights the user's motion. The
  // atomic flag is observed by guidingWatchLoop() which then calls
  // /controller_manager/switch_controller to deactivate the trajectory
  // controller — that way, when GUIDING ends, JTC.on_activate samples
  // actual as the new hold pose instead of jerking back to the old
  // trajectory target.
  const bool guiding =
    (hw_franka_robot_state_.robot_mode == franka::RobotMode::kGuiding);
  in_guiding_.store(guiding);
  if (guiding) {
    auto effort_zero = hw_commands_joint_effort;     effort_zero.fill(0.0);
    auto vel_zero    = hw_commands_joint_velocity;   vel_zero.fill(0.0);
    auto pos_hold    = hw_franka_robot_state_.q;     // hold at actual
    robot_->write(effort_zero, pos_hold, vel_zero,
                  hw_commands_cartesian_position,
                  hw_commands_cartesian_velocity);
    return hardware_interface::return_type::OK;
  }
  robot_->write(hw_commands_joint_effort,
                hw_commands_joint_position,
                hw_commands_joint_velocity,
                hw_commands_cartesian_position,
                hw_commands_cartesian_velocity);
  //RCLCPP_INFO(getLogger(), "write end");
  return hardware_interface::return_type::OK;
}

CallbackReturn FrankaHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  if (info_.joints.size() != kNumberOfJoints) {
    RCLCPP_FATAL(getLogger(), "Got %ld joints. Expected %ld.", info_.joints.size(),
                 kNumberOfJoints);
    return CallbackReturn::ERROR;
  }

  for (const auto& joint : info_.joints) {

    // Check number of command interfaces
    if (joint.command_interfaces.size() != 3) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has %ld command interfaces found. 3 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    // Check that the interfaces are named correctly
    for (const auto & cmd_interface : joint.command_interfaces){
      if (cmd_interface.name != hardware_interface::HW_IF_EFFORT &&   // Effort "effort"
          cmd_interface.name != hardware_interface::HW_IF_POSITION && // Joint position "position"
          cmd_interface.name != hardware_interface::HW_IF_VELOCITY && // Joint velocity "velocity"
          cmd_interface.name != "cartesian_position" &&               // Cartesian position
          cmd_interface.name != "cartesian_velocity"){                // Cartesian velocity
        RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected command interface '%s'",
                    joint.name.c_str(), cmd_interface.name.c_str());
        return CallbackReturn::ERROR;
      }
    }

    // Check number of state interfaces
    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has %zu state interfaces found. 3 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_VELOCITY);
    }
    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_EFFORT);
    }
  }
  std::string robot_ip;
  try {
    robot_ip = info_.hardware_parameters.at("robot_ip");
  } catch (const std::out_of_range& ex) {
    RCLCPP_FATAL(getLogger(), "Parameter 'robot_ip' ! set");
    return CallbackReturn::ERROR;
  }
  // Optional arm_id parameter — when two FrankaHardwareInterface instances live
  // in the same controller_manager (per-arm dual-arm setup), this both renames
  // the exported state/command interface prefixes (so the two arms do not clash
  // in ResourceStorage) and namespaces the error_recovery / param service nodes
  // (so the service names do not clash either).
  std::string arm_id_prefix;
  auto it = info_.hardware_parameters.find("arm_id");
  if (it != info_.hardware_parameters.end() && !it->second.empty()) {
    arm_id_ = it->second;
    arm_id_prefix = it->second + "_";
  }
  // Optional controller_name — name of the trajectory controller that
  // owns this arm's command interfaces. When supplied, GUIDING-mode
  // detection (white button held on the hand) auto-deactivates that
  // controller and re-activates it on release, so JTC.on_activate
  // resamples the user's pose as the new hold pose. When empty, the
  // supervisor branch is disabled and only the torque-zero path in
  // write() runs (still useful — the arm becomes immediately compliant
  // — but JTC will jerk back to its old target when GUIDING ends, so
  // setting controller_name is strongly recommended).
  auto cit = info_.hardware_parameters.find("controller_name");
  if (cit != info_.hardware_parameters.end()) {
    controller_name_ = cit->second;
  }
  try {
    RCLCPP_INFO(getLogger(), "Connecting to robot at \"%s\" ...", robot_ip.c_str());
    robot_ = std::make_unique<Robot>(robot_ip, getLogger());
  } catch (const franka::Exception& e) {
    // RCLCPP_FATAL goes through rcutils, which honours
    // RCUTILS_COLORIZED_OUTPUT=1 (set in the launch env) and emits red
    // ANSI escapes before launch processes the line — more reliable
    // than writing escapes manually from a child process, which launch's
    // 'screen' output handler can strip. std::exit(1) then kills
    // ros2_control_node so the launch's on_exit Shutdown can tear the
    // whole stack down (instead of leaving the controller_manager
    // spinning on 'Waiting for robot_description').
    RCLCPP_FATAL(getLogger(),
        "Could not connect to robot at %s: %s",
        robot_ip.c_str(), e.what());
    std::exit(1);
  }
  RCLCPP_INFO(getLogger(), "Successfully connected to robot");

  // Start the service nodes
  error_recovery_service_node_ = std::make_shared<FrankaErrorRecoveryServiceServer>(rclcpp::NodeOptions(), robot_, arm_id_prefix);
  param_service_node_ = std::make_shared<FrankaParamServiceServer>(rclcpp::NodeOptions(), robot_, arm_id_prefix);
  executor_ = std::make_shared<FrankaExecutor>();
  executor_->add_node(error_recovery_service_node_);
  executor_->add_node(param_service_node_);
  

  // Init the cartesian values to 0
  hw_cartesian_positions_.fill({});
  hw_cartesian_velocities_.fill({});
  hw_commands_cartesian_position.fill({});
  hw_commands_cartesian_velocity.fill({});
  return CallbackReturn::SUCCESS;
}

rclcpp::Logger FrankaHardwareInterface::getLogger() {
  return rclcpp::get_logger("FrankaHardwareInterface");
}

hardware_interface::return_type FrankaHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& /*start_interfaces*/,
    const std::vector<std::string>& /*stop_interfaces*/) {

  RCLCPP_INFO(this->getLogger(),"Performing command mode switch");
  std::cout << "Current mode: " << control_mode_ << std::endl;
  if(control_mode_ == ControlMode::None){
    robot_->stopRobot();
    robot_->initializeContinuousReading();
  }
  else if(control_mode_ == ControlMode::JointTorque){
    robot_->stopRobot();
    robot_->initializeTorqueControl();
  }
  else if(control_mode_ == ControlMode::JointPosition){
    robot_->stopRobot();
    robot_->initializeJointPositionControl();
  }
  else if(control_mode_ == ControlMode::JointVelocity){
    robot_->stopRobot();
    robot_->initializeJointVelocityControl();
  }
  else if(control_mode_ == ControlMode::CartesianPose){
    robot_->stopRobot();
    robot_->initializeCartesianPositionControl();
  }
  else if(control_mode_ == ControlMode::CartesianVelocity){
    robot_->stopRobot();
    robot_->initializeCartesianVelocityControl();
  }
  robot_->setControlMode(control_mode_);
  

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FrankaHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  RCLCPP_INFO(this->getLogger(),"Preparing command mode switch");
  for(auto& start_item : start_interfaces){
    RCLCPP_INFO(this->getLogger(),"++ %s", start_item.c_str());
  }
  for(auto& stop_item : stop_interfaces){
    RCLCPP_INFO(this->getLogger(),"-- %s", stop_item.c_str());
  }
  
  bool is_effort;
  bool is_position;
  bool is_velocity;
  bool is_duplicate;
  
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
  //              Handle the stop case first                    //
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//

  ////////////////////////////////////////////////////////////////
  //              Verify that the names are valid               //
  ////////////////////////////////////////////////////////////////
  int stop_type = check_command_mode_type(stop_interfaces);

  if(stop_type == -1){
    RCLCPP_ERROR(this->getLogger(), "Requested stop interfaces do not all have the same type!\n" \
                                    "Please make sure they are either Cartesian or Joint.");
    return hardware_interface::return_type::ERROR;
  }

  if(stop_type != 0){
    // If stop_type is not empty, i.e. 1 or 2
    is_effort = all_of_element_has_string(stop_interfaces, "effort");
    is_position = all_of_element_has_string(stop_interfaces, "position");
    is_velocity = all_of_element_has_string(stop_interfaces, "velocity");
    is_duplicate = (is_effort && is_position) || (is_effort && is_velocity) || (is_velocity && is_position);
    if(!(is_effort || is_position || is_velocity)){
      RCLCPP_ERROR(this->getLogger(), "Requested stop interface is not a supported type!\n" \
                                  "Please make sure they are either effort, position or velocity.");
      return hardware_interface::return_type::ERROR;
    }
    if((is_duplicate)){
      RCLCPP_ERROR(this->getLogger(), "Requested stop interface has a confusing name!\n" \
                                  "Please make sure they are either effort (for joints), position or velocity, only.");
      return hardware_interface::return_type::ERROR;
    }
  }

  switch(stop_type){
    case 1: // stop the joint controllers
      // Make sure there are 7
      if(stop_interfaces.size() != kNumberOfJoints){
        RCLCPP_ERROR(this->getLogger(), "Requested joint stop interface's size is not 7 (got %ld)", stop_interfaces.size());
        return hardware_interface::return_type::ERROR;
      }
      
      for(size_t i = 0 ; i < kNumberOfJoints; i++){
        if(is_effort){
          hw_commands_joint_effort[i] = 0;
        }
        // position command is not reset, since that can be dangerous
        else if(is_velocity){
          hw_commands_joint_velocity[i] = 0;
        }
      }
      control_mode_ = ControlMode::None;
      break;

    case 2: // stop the cartesian controllers
      if(is_position){
        if(stop_interfaces.size() != 16){
          RCLCPP_ERROR(this->getLogger(), "Requested Cartesian position stop interface's size is not 16 (got %ld)", stop_interfaces.size());
          return hardware_interface::return_type::ERROR;
        }
      }
      if(is_velocity){
        if(stop_interfaces.size() != 6){
          RCLCPP_ERROR(this->getLogger(), "Requested Cartesian velocity stop interface's size is not 6 (got %ld)", stop_interfaces.size());
          return hardware_interface::return_type::ERROR;

        }
        // set the commands to zero
        for(int i = 0; i < 6; i++){
          hw_commands_cartesian_velocity[i] = 0;
        }
      }
      control_mode_ = ControlMode::None;
      break;

    default:
      break;
  }

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
  //              Handle the start case                         //
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
  int start_type = check_command_mode_type(start_interfaces);
  
  if(start_type == -1){
    RCLCPP_ERROR(this->getLogger(), "Requested start interfaces do not all have the same type!\n" \
                                    "Please make sure they are either Cartesian or Joint.");
    return hardware_interface::return_type::ERROR;
  }
  ////////////////////////////////////////////////////////////////
  //              Verify that the names are valid               //
  ////////////////////////////////////////////////////////////////
  if(start_type != 0){
    // If start_type is not empty, i.e. 1 or 2
    is_effort = all_of_element_has_string(start_interfaces, "effort");
    is_position = all_of_element_has_string(start_interfaces, "position");
    is_velocity = all_of_element_has_string(start_interfaces, "velocity");
    is_duplicate = (is_effort && is_position) || (is_effort && is_velocity) || (is_velocity && is_position);
    if(!(is_effort || is_position || is_velocity)){
      RCLCPP_ERROR(this->getLogger(), "Requested start interface is not a supported type!\n" \
                                  "Please make sure they are either effort (for joints), position or velocity.");
      return hardware_interface::return_type::ERROR;
    }
    if((is_duplicate)){
      RCLCPP_ERROR(this->getLogger(), "Requested start interface has a confusing name!\n" \
                                  "Please make sure they are either effort (for joints), position or velocity, only.");
      return hardware_interface::return_type::ERROR;
    }
  }

  // Just in case: only allow new controller to be started if the current control mode is NONE
  if(control_mode_ != ControlMode::None){
    RCLCPP_ERROR(this->getLogger(), "Switching between control modes without stopping it first is not supported.\n"\
                                    "Please stop the running controller first.");
    return hardware_interface::return_type::ERROR;
  }

  switch(start_type){
    case 1:
      if(start_interfaces.size() != kNumberOfJoints){
        RCLCPP_ERROR(this->getLogger(), "Requested joint start interface's size is not 7 (got %ld)", start_interfaces.size());
        return hardware_interface::return_type::ERROR;
      }
      if(is_effort){
        control_mode_ = ControlMode::JointTorque;
      }
      if(is_position){
        control_mode_ = ControlMode::JointPosition;
      }
      if(is_velocity){
        control_mode_ = ControlMode::JointVelocity;
      }
      break;

    case 2:

      if(start_interfaces.size() != 16U && start_interfaces.size() != 6U){
        RCLCPP_ERROR(this->getLogger(), "Requested Cartesian start interface's size is not 6 nor 16 (got %ld)", start_interfaces.size());
        return hardware_interface::return_type::ERROR;
      }
      if(is_position){
        control_mode_ = ControlMode::CartesianPose;
      }
      if(is_velocity){
        control_mode_ = ControlMode::CartesianVelocity;
      }
      break;

    default:
      break;
  }

  return hardware_interface::return_type::OK;

}
}  // namespace franka_hardware

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_hardware::FrankaHardwareInterface,
                       hardware_interface::SystemInterface)
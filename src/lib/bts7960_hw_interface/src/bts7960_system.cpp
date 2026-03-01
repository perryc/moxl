#include "bts7960_hw_interface/bts7960_system.hpp"

#include <cmath>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

// pigpio daemon interface (remote GPIO via pigpiod)
#include <pigpiod_if2.h>

namespace bts7960_hw_interface
{

int BTS7960System::get_param_int(
  const hardware_interface::HardwareInfo & info,
  const std::string & name, int default_val)
{
  auto it = info.hardware_parameters.find(name);
  if (it != info.hardware_parameters.end()) {
    return std::stoi(it->second);
  }
  return default_val;
}

double BTS7960System::get_param_double(
  const hardware_interface::HardwareInfo & info,
  const std::string & name, double default_val)
{
  auto it = info.hardware_parameters.find(name);
  if (it != info.hardware_parameters.end()) {
    return std::stod(it->second);
  }
  return default_val;
}

hardware_interface::CallbackReturn BTS7960System::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read GPIO pin assignments from URDF ros2_control params
  left_rpwm_pin_ = get_param_int(info, "left_rpwm_pin", 12);
  left_lpwm_pin_ = get_param_int(info, "left_lpwm_pin", 16);
  left_ren_pin_ = get_param_int(info, "left_ren_pin", 20);
  left_len_pin_ = get_param_int(info, "left_len_pin", 21);

  right_rpwm_pin_ = get_param_int(info, "right_rpwm_pin", 13);
  right_lpwm_pin_ = get_param_int(info, "right_lpwm_pin", 26);
  right_ren_pin_ = get_param_int(info, "right_ren_pin", 23);
  right_len_pin_ = get_param_int(info, "right_len_pin", 24);

  pwm_frequency_ = get_param_int(info, "pwm_frequency", 20000);
  max_duty_cycle_ = get_param_int(info, "max_duty_cycle", 255);
  wheel_radius_ = get_param_double(info, "wheel_radius", 0.15);
  max_velocity_ = get_param_double(info, "max_velocity", 1.5);

  // Validate joint configuration
  if (info.joints.size() != 2) {
    RCLCPP_ERROR(rclcpp::get_logger("BTS7960System"),
      "Expected 2 joints (left_wheel, right_wheel), got %zu", info.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : info.joints) {
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(rclcpp::get_logger("BTS7960System"),
        "Joint '%s' must have exactly one velocity command interface", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 2) {
      RCLCPP_ERROR(rclcpp::get_logger("BTS7960System"),
        "Joint '%s' must have exactly 2 state interfaces (position, velocity)",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("BTS7960System"),
    "Initialized with pins: L(RPWM=%d,LPWM=%d,REN=%d,LEN=%d) "
    "R(RPWM=%d,LPWM=%d,REN=%d,LEN=%d) PWM_freq=%dHz",
    left_rpwm_pin_, left_lpwm_pin_, left_ren_pin_, left_len_pin_,
    right_rpwm_pin_, right_lpwm_pin_, right_ren_pin_, right_len_pin_,
    pwm_frequency_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BTS7960System::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Connect to pigpio daemon (must be running: sudo pigpiod)
  pi_handle_ = pigpio_start(nullptr, nullptr);  // localhost, default port
  if (pi_handle_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("BTS7960System"),
      "Failed to connect to pigpio daemon (pigpiod). Is it running? Error: %d", pi_handle_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("BTS7960System"),
    "Connected to pigpio daemon (handle=%d)", pi_handle_);

  // Configure all pins as outputs
  int pins[] = {
    left_rpwm_pin_, left_lpwm_pin_, left_ren_pin_, left_len_pin_,
    right_rpwm_pin_, right_lpwm_pin_, right_ren_pin_, right_len_pin_
  };
  for (int pin : pins) {
    set_mode(pi_handle_, pin, PI_OUTPUT);
  }

  // Set PWM frequency on all PWM pins
  int pwm_pins[] = {left_rpwm_pin_, left_lpwm_pin_, right_rpwm_pin_, right_lpwm_pin_};
  for (int pin : pwm_pins) {
    set_PWM_range(pi_handle_, pin, max_duty_cycle_);
    set_PWM_frequency(pi_handle_, pin, pwm_frequency_);
    set_PWM_dutycycle(pi_handle_, pin, 0);  // start with motors stopped
  }

  // Start with motors disarmed (enable pins LOW)
  emergency_stop();

  // Reset state
  left_position_ = 0.0;
  left_velocity_ = 0.0;
  right_position_ = 0.0;
  right_velocity_ = 0.0;
  left_velocity_cmd_ = 0.0;
  right_velocity_cmd_ = 0.0;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BTS7960System::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("BTS7960System"), "Arming motors");
  arm_motors();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BTS7960System::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("BTS7960System"), "E-STOP: Disarming motors");
  emergency_stop();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BTS7960System::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  emergency_stop();

  if (pi_handle_ >= 0) {
    // Set all PWM to 0
    int pwm_pins[] = {left_rpwm_pin_, left_lpwm_pin_, right_rpwm_pin_, right_lpwm_pin_};
    for (int pin : pwm_pins) {
      set_PWM_dutycycle(pi_handle_, pin, 0);
    }

    pigpio_stop(pi_handle_);
    pi_handle_ = -1;
    RCLCPP_INFO(rclcpp::get_logger("BTS7960System"), "Disconnected from pigpio daemon");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
BTS7960System::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Left wheel
  state_interfaces.emplace_back(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &left_position_);
  state_interfaces.emplace_back(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &left_velocity_);

  // Right wheel
  state_interfaces.emplace_back(
    info_.joints[1].name, hardware_interface::HW_IF_POSITION, &right_position_);
  state_interfaces.emplace_back(
    info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &right_velocity_);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
BTS7960System::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &left_velocity_cmd_);
  command_interfaces.emplace_back(
    info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &right_velocity_cmd_);

  return command_interfaces;
}

hardware_interface::return_type BTS7960System::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Open-loop estimation: assume the motor achieves the commanded velocity.
  // This is used by diff_drive_controller for internal bookkeeping.
  // Actual position is determined by GPS (robot_localization), not wheel odometry.
  double dt = period.seconds();

  left_velocity_ = left_velocity_cmd_;
  right_velocity_ = right_velocity_cmd_;

  // Integrate velocity to get position (in radians)
  left_position_ += left_velocity_ * dt;
  right_position_ += right_velocity_ * dt;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type BTS7960System::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (pi_handle_ < 0) {
    return hardware_interface::return_type::ERROR;
  }

  // diff_drive_controller sends velocity in rad/s for each wheel joint.
  // Convert to linear velocity at wheel rim: v = omega * radius
  // Then map to PWM duty cycle.
  set_motor_pwm(left_rpwm_pin_, left_lpwm_pin_, left_velocity_cmd_);
  set_motor_pwm(right_rpwm_pin_, right_lpwm_pin_, right_velocity_cmd_);

  return hardware_interface::return_type::OK;
}

void BTS7960System::set_motor_pwm(int rpwm_pin, int lpwm_pin, double velocity_rad_s)
{
  // Convert angular velocity (rad/s) to linear velocity (m/s) at wheel rim
  double linear_velocity = velocity_rad_s * wheel_radius_;

  // Map linear velocity to duty cycle
  // Clamp to max_velocity_ to avoid exceeding PWM range
  double fraction = std::abs(linear_velocity) / max_velocity_;
  fraction = std::min(fraction, 1.0);

  int duty = static_cast<int>(fraction * max_duty_cycle_);

  if (linear_velocity >= 0.0) {
    // Forward: RPWM active, LPWM off
    // Never have both PWM pins active simultaneously (BTS7960 protection)
    set_PWM_dutycycle(pi_handle_, lpwm_pin, 0);
    set_PWM_dutycycle(pi_handle_, rpwm_pin, duty);
  } else {
    // Reverse: LPWM active, RPWM off
    set_PWM_dutycycle(pi_handle_, rpwm_pin, 0);
    set_PWM_dutycycle(pi_handle_, lpwm_pin, duty);
  }
}

void BTS7960System::emergency_stop()
{
  if (pi_handle_ < 0) return;

  // Pull all enable pins LOW — immediately disables both H-bridges
  gpio_write(pi_handle_, left_ren_pin_, 0);
  gpio_write(pi_handle_, left_len_pin_, 0);
  gpio_write(pi_handle_, right_ren_pin_, 0);
  gpio_write(pi_handle_, right_len_pin_, 0);

  // Also zero out PWM
  set_PWM_dutycycle(pi_handle_, left_rpwm_pin_, 0);
  set_PWM_dutycycle(pi_handle_, left_lpwm_pin_, 0);
  set_PWM_dutycycle(pi_handle_, right_rpwm_pin_, 0);
  set_PWM_dutycycle(pi_handle_, right_lpwm_pin_, 0);
}

void BTS7960System::arm_motors()
{
  if (pi_handle_ < 0) return;

  // Set all enable pins HIGH — both half-bridges enabled on each board
  gpio_write(pi_handle_, left_ren_pin_, 1);
  gpio_write(pi_handle_, left_len_pin_, 1);
  gpio_write(pi_handle_, right_ren_pin_, 1);
  gpio_write(pi_handle_, right_len_pin_, 1);
}

}  // namespace bts7960_hw_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(bts7960_hw_interface::BTS7960System,
  hardware_interface::SystemInterface)

#pragma once

#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace bts7960_hw_interface
{

/// Hardware interface for two BTS7960 H-bridge motor drivers via Raspberry Pi GPIO.
///
/// Drives a differential-drive robot with two wheelchair motors.
/// Each BTS7960 board has:
///   - RPWM: forward PWM input
///   - LPWM: reverse PWM input
///   - R_EN: right-side enable (HIGH to enable)
///   - L_EN: left-side enable (HIGH to enable)
///
/// The enable pins serve as a software e-stop: pulling them LOW
/// immediately disables the H-bridge regardless of PWM state.
///
/// Uses pigpio daemon (pigpiod) for GPIO access — allows non-root
/// operation and provides DMA-based software PWM with microsecond accuracy.
class BTS7960System : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(BTS7960System)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // pigpio daemon handle (-1 if not connected)
  int pi_handle_{-1};

  // GPIO pin assignments for left motor (BTS7960 #1)
  int left_rpwm_pin_{12};
  int left_lpwm_pin_{16};
  int left_ren_pin_{20};
  int left_len_pin_{21};

  // GPIO pin assignments for right motor (BTS7960 #2)
  int right_rpwm_pin_{13};
  int right_lpwm_pin_{26};
  int right_ren_pin_{23};
  int right_len_pin_{24};

  // PWM parameters
  int pwm_frequency_{20000};  // Hz (BTS7960 supports up to 25kHz)
  int max_duty_cycle_{255};   // pigpio PWM range (0-255)

  // Max velocity from config (m/s at wheel rim)
  double max_velocity_{1.5};

  // Joint state (open-loop estimated from commands)
  double left_position_{0.0};
  double left_velocity_{0.0};
  double right_position_{0.0};
  double right_velocity_{0.0};

  // Joint commands (velocity in rad/s from diff_drive_controller)
  double left_velocity_cmd_{0.0};
  double right_velocity_cmd_{0.0};

  // Wheel radius for velocity conversion
  double wheel_radius_{0.15};

  /// Set PWM on a single motor channel.
  /// @param rpwm_pin Forward PWM GPIO pin
  /// @param lpwm_pin Reverse PWM GPIO pin
  /// @param velocity Desired velocity (positive = forward, negative = reverse)
  void set_motor_pwm(int rpwm_pin, int lpwm_pin, double velocity);

  /// Pull all enable pins LOW — immediate motor stop.
  void emergency_stop();

  /// Set all enable pins HIGH — arm motors.
  void arm_motors();

  /// Read an integer parameter from hardware info, with a default value.
  int get_param_int(const hardware_interface::HardwareInfo & info,
                    const std::string & name, int default_val);

  /// Read a double parameter from hardware info, with a default value.
  double get_param_double(const hardware_interface::HardwareInfo & info,
                          const std::string & name, double default_val);
};

}  // namespace bts7960_hw_interface

#ifndef MOCK_SERIAL_ARM__MOCK_HARDWARE_HPP_
#define MOCK_SERIAL_ARM__MOCK_HARDWARE_HPP_

#include <vector>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"

namespace mock_serial_arm
{

class MockSerialHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MockSerialHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  std::vector<double> hw_positions_;
  std::vector<double> hw_commands_;
};

}  // namespace mock_serial_arm

#endif  // MOCK_SERIAL_ARM__MOCK_HARDWARE_HPP_

#include "mock_serial_arm/mock_hardware.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace mock_serial_arm
{

hardware_interface::CallbackReturn MockSerialHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MockSerialHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_positions_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MockSerialHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_commands_[i]);
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn MockSerialHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MockSerialHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MockSerialHardware::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  // Simulate hardware: position follows command
  hw_positions_ = hw_commands_;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MockSerialHardware::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

}  // namespace mock_serial_arm

PLUGINLIB_EXPORT_CLASS(
  mock_serial_arm::MockSerialHardware,
  hardware_interface::SystemInterface)

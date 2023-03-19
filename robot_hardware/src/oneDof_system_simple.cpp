#include "robot_hardware/oneDof_system_simple.hpp"
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace robot_hardware
{
  
  CallbackReturn OneDofSimplePosition::on_init(const hardware_interface::HardwareInfo & info) 
  {

    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"), "Problem");
      return CallbackReturn::ERROR;
    }
    for (const  hardware_interface::ComponentInfo & joint : info_.joints)
    {
      RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"), "Joint : %s", joint.name.c_str());
    }
    hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn OneDofSimplePosition::on_configure(const rclcpp_lifecycle::State & previous_state) 
  {
    (void)previous_state;
    for (uint i = 0; i < hw_states_.size(); i++)
    {
        hw_states_[i] = 0;
        hw_commands_[i] = 0;
    }
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> OneDofSimplePosition::export_state_interfaces() 
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> OneDofSimplePosition::export_command_interfaces() 
  {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
        }

        return command_interfaces;
  }

  CallbackReturn OneDofSimplePosition::on_activate(const rclcpp_lifecycle::State & previous_state) 
  {
    (void)previous_state;
    for (uint i = 0; i < hw_states_.size(); i++)
    {
      hw_commands_[i] = hw_states_[i];
    }

    RCLCPP_INFO(rclcpp::get_logger("Activate"), "We activated bby");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn OneDofSimplePosition::on_deactivate(const rclcpp_lifecycle::State & previous_state)
  {
    (void)previous_state;
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type OneDofSimplePosition::read() 
  {
    for (uint i = 0; i < hw_states_.size(); i++)
    {
      // Simulate RRBot's movement
      hw_states_[i] = hw_states_[i] + (hw_commands_[i] - hw_states_[i]);
      //RCLCPP_INFO(
      //  rclcpp::get_logger("OneDofSimplePosition"), "Got state %.5f for joint %d!",
      //  hw_states_[i], i);
    }
    return hardware_interface::return_type::OK;

  }

  hardware_interface::return_type OneDofSimplePosition::write() 
  {
    return hardware_interface::return_type::OK;
  }

} // namespace robot_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    robot_hardware::OneDofSimplePosition, hardware_interface::SystemInterface
)

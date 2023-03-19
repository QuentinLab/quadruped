#ifndef ROBOT_HARDWARE_SIMPLE_POSITION_HPP_
#define ROBOT_HARDWARE_SIMPLE_POSITION_HPP_

#include <vector>
#include <memory>
#include <string>

#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace robot_hardware
{
class OneDofSimplePosition : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read() override;

  hardware_interface::return_type write() override;
  
private:
  // Parameters for the RRBot simulation
  double test_param_;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
};

}  // namespace ros2_control_demo_hardware

#endif
#ifndef SO_ARM_101_HARDWARE__SO_ARM_101_HARDWARE_HPP_
#define SO_ARM_101_HARDWARE__SO_ARM_101_HARDWARE_HPP_

#include <vector>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "SMS_STS.h"

namespace so_arm_101_hardware
{
    class SoArm101HardwareInterface : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(SoArm101HardwareInterface)

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        SMS_STS servo_;
        std::string serial_port_;

        std::vector<double> hw_positions_;
        std::vector<double> hw_commands_;
        std::vector<double> hw_velocities_;

        std::vector<uint8_t> servo_ids_;
        std::vector<std::string> joint_names_;

        std::vector<double> zero_offset_ticks_;
        std::vector<double> direction_;
    };
} // namespace so_arm_101_hardware

#endif // SO_ARM_101_HARDWARE__SO_ARM_101_HARDWARE_HPP_
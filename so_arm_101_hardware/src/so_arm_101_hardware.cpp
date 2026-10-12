#include "so_arm_101_hardware/so_arm_101_hardware.hpp"

#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace so_arm_101_hardware
{
    namespace
    {
        constexpr double kTicksPerRev = 4096.0;
        constexpr double kTicksPerRad = kTicksPerRev / (2.0 * M_PI);
        constexpr uint16_t kDefaultSpeed = 2000;
        constexpr uint8_t kDefaultAcc = 10;

        rclcpp::Logger logger() { return rclcpp::get_logger("so_arm_101_hardware"); }
    } // namespace

    hardware_interface::CallbackReturn SoArm101HardwareInterface::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        const auto num_joints = info_.joints.size();
        hw_positions_.assign(num_joints, std::numeric_limits<double>::quiet_NaN());
        hw_commands_.assign(num_joints, std::numeric_limits<double>::quiet_NaN());
        hw_velocities_.assign(num_joints, 0.0);
        joint_names_.resize(num_joints);
        servo_ids_.resize(num_joints);
        zero_offset_ticks_.assign(num_joints, 0.0);
        direction_.assign(num_joints, 1.0);

        for (size_t i = 0; i < num_joints; ++i)
        {
            joint_names_[i] = info_.joints[i].name;
            const auto &params = info_.joints[i].parameters;

            auto id_it = params.find("servo_id");
            if (id_it == params.end())
            {
                RCLCPP_ERROR(logger(), "Joint '%s' is missing the 'servo_id' param", joint_names_[i].c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
            servo_ids_[i] = static_cast<uint8_t>(std::stoi(id_it->second));

            if (auto it = params.find("zero_offset_ticks"); it != params.end())
            {
                zero_offset_ticks_[i] = std::stod(it->second);
            }
            if (auto it = params.find("direction"); it != params.end())
            {
                direction_[i] = std::stod(it->second);
            }
        }

        auto port_it = info_.hardware_parameters.find("serial_port");
        if (port_it == info_.hardware_parameters.end())
        {
            RCLCPP_ERROR(logger(), "Missing 'serial_port' hardware param");
            return hardware_interface::CallbackReturn::ERROR;
        }
        serial_port_ = port_it->second;

        if (!servo_.begin(1000000, serial_port_.c_str()))
        {
            RCLCPP_ERROR(logger(), "Failed to open serial port '%s'", serial_port_.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(logger(), "so_arm_101_hardware initialized on '%s'", serial_port_.c_str());
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> SoArm101HardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
            state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> SoArm101HardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            command_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
        }
        return command_interfaces;
    }

    hardware_interface::CallbackReturn SoArm101HardwareInterface::on_activate(const rclcpp_lifecycle::State &)
    {
        for (size_t i = 0; i < servo_ids_.size(); ++i)
        {
            servo_.EnableTorque(servo_ids_[i], 1);

            int raw = servo_.ReadPos(servo_ids_[i]);
            if (raw == -1)
            {
                RCLCPP_ERROR(logger(), "Failed to read initial position for joint '%s'", joint_names_[i].c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
            double rad = direction_[i] * (raw - zero_offset_ticks_[i]) / kTicksPerRad;
            hw_positions_[i] = rad;
            hw_commands_[i] = rad;
        }
        RCLCPP_INFO(logger(), "so_arm_101_hardware activated, torque enabled");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn SoArm101HardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
    {
        for (const auto id : servo_ids_)
        {
            servo_.EnableTorque(id, 0);
        }
        RCLCPP_INFO(logger(), "so_arm_101_hardware deactivated, torque disabled");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type SoArm101HardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
    {
        for (size_t i = 0; i < servo_ids_.size(); ++i)
        {
            int raw = servo_.ReadPos(servo_ids_[i]);
            if (raw == -1)
            {
                continue; // keep last known value on a failed read
            }
            hw_positions_[i] = direction_[i] * (raw - zero_offset_ticks_[i]) / kTicksPerRad;
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type SoArm101HardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
    {
        for (size_t i = 0; i < servo_ids_.size(); ++i)
        {
            if (std::isnan(hw_commands_[i]))
            {
                continue;
            }
            s16 ticks = static_cast<s16>(zero_offset_ticks_[i] + direction_[i] * hw_commands_[i] * kTicksPerRad);
            servo_.WritePosEx(servo_ids_[i], ticks, kDefaultSpeed, kDefaultAcc);
        }
        return hardware_interface::return_type::OK;
    }

} // namespace so_arm_101_hardware

PLUGINLIB_EXPORT_CLASS(so_arm_101_hardware::SoArm101HardwareInterface, hardware_interface::SystemInterface)
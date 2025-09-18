#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <vector>
#include <thread>

namespace irrobot_hardware_interface
{
class IrrobotSystemInterface : public hardware_interface::SystemInterface
{
public:
    rclcpp::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override
    {
        if (hardware_interface::SystemInterface::on_init(info) != rclcpp::CallbackReturn::SUCCESS)
            return rclcpp::CallbackReturn::ERROR;

        hw_positions_.resize(info_.joints.size(), 0.0);
        hw_velocities_.resize(info_.joints.size(), 0.0);
        hw_commands_.resize(info_.joints.size(), 0.0);

        // Inicializa nó ROS em thread separada
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<rclcpp::Node>("irrobot_hw_interface");

        cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("diff_drive_controller/cmd_vel", 10);

        state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states_hw", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg)
            {
                for (size_t i = 0; i < hw_positions_.size() && i < msg->position.size(); ++i)
                {
                    hw_positions_[i] = msg->position[i];
                    hw_velocities_[i] = msg->velocity[i];
                }
            });

        executor_thread_ = std::thread([this]() { rclcpp::spin(node_); });

        RCLCPP_INFO(rclcpp::get_logger("IrrobotSystemInterface"), "Hardware com comunicação real inicializado.");

        return rclcpp::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
            state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
        }
        return command_interfaces;
    }

    hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
    {
        // Estados já são atualizados via callback do subscriber
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
    {
        // Envia velocidades como Twist para outro nó (Python)
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = (hw_commands_[0] + hw_commands_[1]) / 2.0;  // simplificação
        msg.angular.z = (hw_commands_[1] - hw_commands_[0]) / 0.5; // ajuste da base (track_width)

        cmd_pub_->publish(msg);
        return hardware_interface::return_type::OK;
    }

    ~IrrobotSystemInterface()
    {
        rclcpp::shutdown();
        if (executor_thread_.joinable())
            executor_thread_.join();
    }

private:
    std::vector<double> hw_positions_, hw_velocities_, hw_commands_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
    std::thread executor_thread_;
};
} // namespace irrobot_hardware_interface

PLUGINLIB_EXPORT_CLASS(irrobot_hardware_interface::IrrobotSystemInterface, hardware_interface::SystemInterface)

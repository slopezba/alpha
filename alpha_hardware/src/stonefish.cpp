#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <limits> // Para std::numeric_limits

using CallbackReturn = hardware_interface::CallbackReturn;
using namespace hardware_interface;

class alpha_stonefish_hardware : public hardware_interface::SystemInterface {
public:
    // Inicialización del plugin
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override {
        if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }

        // Inicializar vectores para estados y comandos
        joint_positions_.resize(info.joints.size(), 0.0);
        joint_velocities_.resize(info.joints.size(), 0.0);
        joint_efforts_.resize(info.joints.size(), 0.0);
        joint_position_commands_.resize(info.joints.size(), std::numeric_limits<double>::quiet_NaN());
        joint_velocity_commands_.resize(info.joints.size(), std::numeric_limits<double>::quiet_NaN());
        control_modes_.resize(info.joints.size(), ControlMode::kNone);
        joint_names_.resize(info.joints.size());

        for (size_t i = 0; i < info.joints.size(); ++i) {
            joint_names_[i] = info.joints[i].name;
        }

        // Configurar ROS 2: nodo, subscripción y publicación
        node_ = rclcpp::Node::make_shared("alpha_stonefish_hardware");
        joint_command_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
            "/peacetolero/alpha/desired_joint_states", 10);

        joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "/peacetolero/joint_states", 10,
            std::bind(&alpha_stonefish_hardware::joint_state_callback, this, std::placeholders::_1));

        return CallbackReturn::SUCCESS;
    }

    // Exportar las interfaces de estado
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
        std::vector<StateInterface> state_interfaces;
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            state_interfaces.emplace_back(StateInterface(joint_names_[i], HW_IF_POSITION, &joint_positions_[i]));
            state_interfaces.emplace_back(StateInterface(joint_names_[i], HW_IF_VELOCITY, &joint_velocities_[i]));
            state_interfaces.emplace_back(StateInterface(joint_names_[i], HW_IF_EFFORT, &joint_efforts_[i]));
        }
        return state_interfaces;
    }

    // Exportar las interfaces de comando
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
        std::vector<CommandInterface> command_interfaces;
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            command_interfaces.emplace_back(CommandInterface(joint_names_[i], HW_IF_POSITION, &joint_position_commands_[i]));
            command_interfaces.emplace_back(CommandInterface(joint_names_[i], HW_IF_VELOCITY, &joint_velocity_commands_[i]));
        }
        return command_interfaces;
    }

    // Leer el estado de las articulaciones desde el simulador
    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override {
        // Procesar los callbacks pendientes
        rclcpp::spin_some(node_);

        // // Imprimir el estado actual para depuración
        // for (size_t i = 0; i < joint_names_.size(); ++i) {
        //     RCLCPP_INFO(node_->get_logger(), "Joint %s: Position=%.2f, Velocity=%.2f, Effort=%.2f",
        //                 joint_names_[i].c_str(), joint_positions_[i], joint_velocities_[i], joint_efforts_[i]);
        // }
        return return_type::OK;
    }

    // Escribir comandos al simulador
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = node_->get_clock()->now();
        msg.name = joint_names_;

        bool has_valid_position = false;
        bool has_valid_velocity = false;

        for (size_t i = 0; i < control_modes_.size(); ++i) {
            if (control_modes_[i] == ControlMode::kPosition && !std::isnan(joint_position_commands_[i])) {
                has_valid_position = true;
            }
            if (control_modes_[i] == ControlMode::kVelocity && !std::isnan(joint_velocity_commands_[i])) {
                has_valid_velocity = true;
            }
        }

        // Agregar comandos según los controladores activos y válidos
        if (has_valid_position) {
            msg.position = joint_position_commands_;
        }
        if (has_valid_velocity) {
            msg.velocity = joint_velocity_commands_;
        }

        // Solo publicar si hay comandos válidos
        if (has_valid_position || has_valid_velocity) {
            joint_command_pub_->publish(msg);
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type prepare_command_mode_switch(
        const std::vector<std::string> &start_interfaces,
        const std::vector<std::string> &stop_interfaces) override {

        for (const auto &interface : stop_interfaces) {
            for (size_t i = 0; i < joint_names_.size(); ++i) {
                if (interface == joint_names_[i] + "/" + HW_IF_POSITION) {
                    control_modes_[i] = ControlMode::kNone;
                } else if (interface == joint_names_[i] + "/" + HW_IF_VELOCITY) {
                    control_modes_[i] = ControlMode::kNone;
                    joint_velocity_commands_[i] = 0.0; // Configurar velocidad a cero al detenerse
                    auto msg = sensor_msgs::msg::JointState();
                    msg.header.stamp = node_->get_clock()->now();
                    msg.name = joint_names_;
                    msg.velocity = joint_velocity_commands_;
                    joint_command_pub_->publish(msg);
                }
            }
        }

        for (const auto &interface : start_interfaces) {
            for (size_t i = 0; i < joint_names_.size(); ++i) {
                if (interface == joint_names_[i] + "/" + HW_IF_POSITION) {
                    control_modes_[i] = ControlMode::kPosition;
                } else if (interface == joint_names_[i] + "/" + HW_IF_VELOCITY) {
                    control_modes_[i] = ControlMode::kVelocity;
                    joint_velocity_commands_[i] = 0.0; // Configurar velocidad a cero al detenerse
                    auto msg = sensor_msgs::msg::JointState();
                    msg.header.stamp = node_->get_clock()->now();
                    msg.name = joint_names_;
                    msg.velocity = joint_velocity_commands_;
                    joint_command_pub_->publish(msg);
                }
            }
        }

        return hardware_interface::return_type::OK;
    }

private:
    enum class ControlMode { kNone, kPosition, kVelocity };

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    std::vector<std::string> joint_names_;
    std::vector<double> joint_positions_, joint_velocities_, joint_efforts_;
    std::vector<double> joint_position_commands_, joint_velocity_commands_;
    std::vector<ControlMode> control_modes_;

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // RCLCPP_INFO(node_->get_logger(), "Received joint_states message");
        for (size_t i = 0; i < msg->name.size(); ++i) {
            auto it = std::find(joint_names_.begin(), joint_names_.end(), msg->name[i]);
            if (it != joint_names_.end()) {
                size_t index = std::distance(joint_names_.begin(), it);
                if (i < msg->position.size()) {
                    joint_positions_[index] = msg->position[i];
                }
                if (i < msg->velocity.size()) {
                    joint_velocities_[index] = msg->velocity[i];
                }
                if (i < msg->effort.size()) {
                    joint_efforts_[index] = msg->effort[i];
                }
            }
        }
    }
    
};

// Registrar el plugin con ROS 2
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(alpha_stonefish_hardware, hardware_interface::SystemInterface)

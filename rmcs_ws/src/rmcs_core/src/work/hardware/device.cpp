#include "hardware/device/dr16.hpp"
#include "hardware/device/dji_motor.hpp"
#include "librmcs/client/cboard.hpp"
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware {

class SingleCBoardExample
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::client::CBoard {

public:
    SingleCBoardExample()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , librmcs::client::CBoard{static_cast<int>(get_parameter("usb_pid").as_int())}
        , robot_command_(create_partner_component<RoboCommand>(get_component_name() + "_command", *this))
        , dr16_(*this)
        , m2006_right(*this, *robot_command_, "/example/m2006_right")
        , m2006_left(*this, *robot_command_, "/example/m2006_left")
        , transmit_buffer_(*this, 32)
        , event_thread_([this]() { handle_events(); }) {
            m2006_right.configure(device::DjiMotor::Config{device::DjiMotor::Type::M2006});
            m2006_left.configure(device::DjiMotor::Config{device::DjiMotor::Type::M2006});
        }

    ~SingleCBoardExample() override {
        stop_handling_events();
        event_thread_.join();
    }

    void update() override {
        dr16_.update_status();
        update_motors();
    }

    void command_update() {
        uint16_t can_commands[4];

        can_commands[0] = 0;
        can_commands[1] = 0;
        can_commands[2] = 0;
        can_commands[3] = 0;
        transmit_buffer_.add_can1_transmission(0x1FE, std::bit_cast<uint64_t>(can_commands));

        can_commands[0] = m2006_right.generate_command();
        can_commands[1] = m2006_left.generate_command();
        can_commands[2] = 0;
        can_commands[3] = 0;
        transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

        can_commands[0] = 0;
        can_commands[1] = 0;
        can_commands[2] = 0;
        can_commands[3] = 0;
        transmit_buffer_.add_can2_transmission(0x1FE, std::bit_cast<uint64_t>(can_commands));

        can_commands[0] = 0;
        can_commands[1] = 0;
        can_commands[2] = 0;
        can_commands[3] = 0;
        transmit_buffer_.add_can2_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

        transmit_buffer_.trigger_transmission();
    }

private:
    void update_motors() {
        m2006_right.update_status();
        m2006_left.update_status();
    }
protected:
    // 关于这里，如果你只有一个声明而没有实现，像我注释掉的内容一样，会报错

    // void can1_receive_callback(
    //     uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
    //     uint8_t can_data_length) override;

    // void can2_receive_callback(
    //     uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
    //     uint8_t can_data_length) override;

    // void uart1_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override;

    // void uart2_receive_callback(const std::byte* data, uint8_t length) override;

    // void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override;
    void can1_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {
        if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
            return;

        if (can_id == 0x201) {
            m2006_right.store_status(can_data);
        }
        if (can_id == 0x202) {
            m2006_left.store_status(can_data);
        }
    }
    void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
        dr16_.store_status(uart_data, uart_data_length);
    }

private:
    class RoboCommand : public rmcs_executor::Component {
    public:
        explicit RoboCommand(SingleCBoardExample& robot)
            : robot_(robot) {}

        void update() override { robot_.command_update(); }

        SingleCBoardExample& robot_;
    };
    std::shared_ptr<RoboCommand> robot_command_;

    // device
    device::Dr16 dr16_;

    device::DjiMotor m2006_right;
    device::DjiMotor m2006_left;

    librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
    std::thread event_thread_;
};
} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::SingleCBoardExample, rmcs_executor::Component)
#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <cmath>
#include <limits>

namespace rmcs_core::example {
class RemoteControllerExample
    : public rmcs_executor::Component
    , public rclcpp::Node {

public:
    RemoteControllerExample()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger()) {

        // this->declare_parameter("kp",0.5);
        this->get_parameter("kp", kp_);

        register_input("/remote/joystick/left", remote_left_joystic_);
        register_input("/remote/joystick/right", remote_right_joystic_);
        register_input("/remote/switch/left", remote_left_switch_);
        register_input("/remote/switch/right", remote_right_switch_);

        register_input("/example/m2006_right/velocity",actual_velocity_right_);
        register_input("/example/m2006_left/velocity",actual_velocity_left_);

        register_output("/example/m2006_right/control_velocity", motor_control_velocity_right);
        register_output("/example/m2006_left/control_velocity", motor_control_velocity_left);
    }

    void update() override {
        using namespace rmcs_msgs;
        if ((*remote_left_switch_ == Switch::DOWN || *remote_left_switch_ == Switch::UNKNOWN)
            && (*remote_right_switch_ == Switch::DOWN || *remote_right_switch_ == Switch::UNKNOWN)) {
            // stop all !!
            *motor_control_velocity_right = nan_;
            *motor_control_velocity_left = nan_;
        } else {
            double base_velocity = max_velocity_ * remote_right_joystic_->y();

            double actual_left_ = *actual_velocity_left_;
            double actual_right_ = *actual_velocity_right_;

            double error_left = base_velocity - actual_left_;
            double error_right = base_velocity - actual_right_;
            double sync_error = actual_left_ - actual_right_;

            double compensation_right = kp_ * (error_right + sync_error );
            double compensation_left = kp_ * (error_left - sync_error );

            *motor_control_velocity_right = base_velocity + compensation_right;
            *motor_control_velocity_left = base_velocity + compensation_left;

            RCLCPP_INFO(logger_, "龙门架控制: 目标速度=%.2f, 左电机=%.2f, 右电机=%.2f, 同步误差=%.2f",
                base_velocity, actual_left_, actual_right_, sync_error);
           
        }
    }

private:
    rclcpp::Logger logger_;
    static constexpr double  nan_ = std::numeric_limits<double>::quiet_NaN();
    double max_velocity_ = 30.0;  // 飞镖架最大升降速度
    double kp_;

    InputInterface<rmcs_msgs::Switch> remote_left_switch_;
    InputInterface<rmcs_msgs::Switch> remote_right_switch_;

    InputInterface<Eigen::Vector2d> remote_left_joystic_;
    InputInterface<Eigen::Vector2d> remote_right_joystic_;

    InputInterface<double> actual_velocity_right_;
    InputInterface<double> actual_velocity_left_;

    OutputInterface<double> motor_control_velocity_right;
    OutputInterface<double> motor_control_velocity_left;
};

} // namespace rmcs_core::example

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::example::RemoteControllerExample, rmcs_executor::Component)
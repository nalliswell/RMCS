#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <cmath>

namespace rmcs_core::controller::communication {

class SignalProcessor final
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SignalProcessor()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}{
        RCLCPP_INFO(get_logger(), "SignalProcessor initialized");

        register_input("sin_value", sin_value_);
        register_input("cos_value", cos_value_);
        register_output("sum_output", sum_output_);
        }

        void update() override {
            double sin = *sin_value_;
            double cos = *cos_value_;
            
            double sum = sin + cos;
            *sum_output_=sum;

            static int count = 0;
            if (count++ % 500 == 0) {
                RCLCPP_INFO(get_logger(), "SignalProcessor: sin=%.3f, cos=%.3f, sum=%.3f", 
                           sin, cos, sum);
        }
        }

private:
    OutputInterface<double> sum_output_;
    InputInterface<double> sin_value_;
    InputInterface<double> cos_value_;
};


} // namespace rmcs_core::communication

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::communication::SignalProcessor, rmcs_executor::Component)
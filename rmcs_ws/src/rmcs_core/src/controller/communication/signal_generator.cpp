#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <cmath>

namespace rmcs_core::controller::communication {

class SignalGenerator final
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SignalGenerator()
        : Node{get_component_name(), 
            rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}{
        
        omega_ = get_parameter_or<double>("omega", 6.28);    

        RCLCPP_INFO(get_logger(), "SignalGenerator initialized with omega: %f", omega_);

        register_output("sin_value", sin_value_);
        register_output("cos_value", cos_value_);
    }
    void update() override {

        time_ += 0.001;

        double sin_val = std::sin(omega_ * time_);
        double cos_val = std::cos(omega_ * time_);
        
        *sin_value_= sin_val; 
        *cos_value_=cos_val;  

        static int count = 0;
        if (count++ % 500 == 0) {
            RCLCPP_INFO(get_logger(), "SignalGenerator: sin=%.3f, cos=%.3f", sin_val, cos_val);
        }
    }

private:
    double omega_ = 6.28;
    double time_ = 0.0;

    OutputInterface<double> sin_value_;
    OutputInterface<double> cos_value_;
};


} // namespace rmcs_core::communication

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::communication::SignalGenerator, rmcs_executor::Component)
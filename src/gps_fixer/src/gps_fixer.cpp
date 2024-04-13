#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

class GpsFixer : public rclcpp::Node
{
public:
    GpsFixer() : Node("gps_fixer")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("fix_out", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "fix_in", 10, std::bind(&GpsFixer::topic_callback, this, std::placeholders::_1));

    
        // error_scalar parameter
        {
            auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
            param_desc.description = "Multiplier applied to standard deviation  calculating covariances. Setting this to 1 will make the standard deviation equal to the error. Higher values will make localization less sensitive to GPS errors.";
            param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
            this->declare_parameter("error_scalar", 1.0, param_desc);
        }



    }

private:
    void topic_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) const
    {
        double fixed_std = 10.0; // report 10 meters minimum standard deviation
        // make a copy of msg
        auto fixed_msg = std::make_shared<sensor_msgs::msg::NavSatFix>(*msg);
        auto cov = msg->position_covariance[0];

        if(cov < 0.2) {
            fixed_std = 10;
        } else if (cov < 1.0) {
            fixed_std = 30;
        } else if (cov < 10) {
            fixed_std = 100;
        } else {
            fixed_std = 99990000;
        }



        // multiply by error_scalar parameter
        double error_scalar;
        this->get_parameter("error_scalar", error_scalar);
        fixed_std *= error_scalar;

        auto fixed_cov = fixed_std* fixed_std;
        fixed_msg->position_covariance = {fixed_cov, 0, 0, 0, fixed_cov, 0, 0, 0, fixed_cov};
        publisher_->publish(*fixed_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsFixer>());
    rclcpp::shutdown();
    return 0;
}
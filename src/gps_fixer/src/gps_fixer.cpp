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
    }

private:
    void topic_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) const
    {
        double min_std = 10.0; // report 10 meters minimum standard deviation
        // make a copy of msg
        auto fixed_msg = std::make_shared<sensor_msgs::msg::NavSatFix>(*msg);
        auto cov = msg->position_covariance[0];
        if (cov > 0) {
            auto std_dev = sqrt(cov);
            std_dev = std::max(std_dev, min_std);
            auto fixed_cov = std_dev * std_dev;
            fixed_msg->position_covariance = {fixed_cov, 0, 0, 0, fixed_cov, 0, 0, 0, fixed_cov};
            publisher_->publish(*fixed_msg);
        }
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
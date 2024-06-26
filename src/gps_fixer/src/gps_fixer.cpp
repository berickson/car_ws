#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nmea_msgs/msg/sentence.hpp"

class GpsFixer : public rclcpp::Node
{
public:
    GpsFixer() : Node("gps_fixer")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("fix_out", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "fix_in", 10, std::bind(&GpsFixer::topic_callback, this, std::placeholders::_1));
        

        sentence_subscription_ = this->create_subscription<nmea_msgs::msg::Sentence>(
            "sentence_in",rclcpp::SensorDataQoS(), 
            std::bind(&GpsFixer::sentence_callback, this, std::placeholders::_1));

        sentence_publisher_ = this->create_publisher<nmea_msgs::msg::Sentence>("sentence_out", 10);

    
        // error_factor parameter
        {
            auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
            param_desc.description = "Multiplier applied to standard deviation for calculating covariances. Setting this to 1 will make the standard deviation equal to the error. Higher values will make localization less sensitive to GPS errors.";
            param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
            this->declare_parameter("error_factor", 1.0, param_desc);
        }

        // min_std parameter
        {
            auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
            param_desc.description = "Minimum standard deviation to report in meters.";
            param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
            this->declare_parameter("min_std", 10.0, param_desc);
        }
    }

private:
    void sentence_callback(const nmea_msgs::msg::Sentence::SharedPtr msg) const
    {

        // copy msg
        auto fixed_msg = std::make_shared<nmea_msgs::msg::Sentence>(*msg);
        sentence_publisher_->publish(*fixed_msg);
    }

    void topic_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) const
    {
        double min_std = get_parameter("min_std").as_double();
        // make a copy of msg
        auto fixed_msg = std::make_shared<sensor_msgs::msg::NavSatFix>(*msg);
        auto cov = msg->position_covariance[0];
        double original_std = std::sqrt(cov);

        double fixed_std = std::max(min_std, original_std);


        // multiply by error_factor parameter
        double error_factor;
        this->get_parameter("error_factor", error_factor);
        fixed_std *= error_factor;

        auto fixed_cov = fixed_std * fixed_std;
        fixed_msg->position_covariance = {fixed_cov, 0, 0, 0, fixed_cov, 0, 0, 0, fixed_cov};
        publisher_->publish(*fixed_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;

    rclcpp::Subscription<nmea_msgs::msg::Sentence>::SharedPtr sentence_subscription_;
    rclcpp::Publisher<nmea_msgs::msg::Sentence>::SharedPtr sentence_publisher_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsFixer>());
    rclcpp::shutdown();
    return 0;
}
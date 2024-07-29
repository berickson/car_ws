#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"


class TeleopSteam : public rclcpp::Node
{
public:
    TeleopSteam() : Node("teleop_steam")
    {
        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&TeleopSteam::joy_callback, this, std::placeholders::_1));

        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

private:
    

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) const
    {
        auto twist = std::make_shared<geometry_msgs::msg::Twist>();
        auto throttle_percentage = msg->axes[5];
        auto brake_percentage = msg->axes[4];
        auto steering = msg->axes[2];

        auto max_forward_speed = 3.0;
        auto max_reverse_speed = -1.0;
        auto max_curvature = M_PI; // radians per meter

        if(throttle_percentage > 0 && brake_percentage > 0)
        {
            twist->linear.x = 0;
        }
        else if(throttle_percentage > 0)
        {
            twist->linear.x = throttle_percentage * max_forward_speed;
        }
        else if(brake_percentage > 0)
        {
            twist->linear.x = brake_percentage * max_reverse_speed;
        }
        else
        {
            twist->linear.x = 0.01;
        }



        twist->angular.z = steering * max_curvature * twist->linear.x;



        twist_publisher_->publish(*twist);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopSteam>());
    rclcpp::shutdown();
    return 0;
}
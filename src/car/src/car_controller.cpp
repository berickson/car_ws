#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      
      subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "micro_ros_platformio_node_publisher", rclcpp::SensorDataQoS(), std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) const
    {
      static int i = 0;

      if (i % 100  == 0) {
        RCLCPP_INFO(this->get_logger(), "callback #%d I heard: '%d'",i, msg->data);
      }
      ++i;

    }
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "car_msgs/msg/update.hpp"
using std::placeholders::_1;

class CarControllerNode : public rclcpp::Node
{
  public:
    CarControllerNode()
    : Node("car_controller")
    {
      
      subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "micro_ros_platformio_node_publisher", rclcpp::SensorDataQoS(), std::bind(&CarControllerNode::topic_callback, this, _1));
      car_update_subscription_ = this->create_subscription<car_msgs::msg::Update>(
      "car_update", rclcpp::SensorDataQoS(), std::bind(&CarControllerNode::car_update_topic_callback, this, _1));
    }

  private:
    void car_update_topic_callback(const car_msgs::msg::Update::SharedPtr msg) const
    {
      static int i = 0;

      if (i % 100  == 0) {
        RCLCPP_INFO(this->get_logger(), "car_update callback #%d ms: '%d'",i, msg->ms);
      }
      ++i;

    }

    void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) const
    {
      static int i = 0;

      if (i % 100  == 0) {
        RCLCPP_INFO(this->get_logger(), "callback #%d I heard: '%d'",i, msg->data);
      }
      ++i;

    }
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    rclcpp::Subscription<car_msgs::msg::Update>::SharedPtr car_update_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CarControllerNode>());
  rclcpp::shutdown();
  return 0;
}
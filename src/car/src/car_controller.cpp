#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "car_msgs/msg/update.hpp"
using std::placeholders::_1;

class CarControllerNode : public rclcpp::Node
{
  public:
    CarControllerNode()
    : Node("car_controller")
    {
      car_update_subscription_ = this->create_subscription<car_msgs::msg::Update>(
      "car/update", rclcpp::SensorDataQoS(), std::bind(&CarControllerNode::car_update_topic_callback, this, _1));
    }

  private:
    void car_update_topic_callback(const car_msgs::msg::Update::SharedPtr msg) const
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "Running and receiving car update messages");
    }

    rclcpp::Subscription<car_msgs::msg::Update>::SharedPtr car_update_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CarControllerNode>());
  rclcpp::shutdown();
  return 0;
}
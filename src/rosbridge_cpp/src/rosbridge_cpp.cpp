#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "json_encoder.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class RosbridgeCppNode : public rclcpp::Node
{
  public:
    RosbridgeCppNode()
    : Node("rosbridge_cpp"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&RosbridgeCppNode::timer_callback, this));

      generic_subscription_ = this->create_generic_subscription("car/update", "car_msgs/msg/Update", rclcpp::SensorDataQoS(), std::bind(&RosbridgeCppNode::generic_callback, this, _1));
      RCLCPP_INFO(this->get_logger(), "Node done creating subsciption");
    }

  private:

    void generic_callback(std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) {


      std::string message_identifier = "car/update";
      std::string message_type = "car_msgs/msg/Update";


      static bool first_time = true;
      if(first_time) {
        json_encoder_.set_message_type(message_type);
        first_time = false;
      }

      std::stringstream ss_json;
      json_encoder_.stream_json(ss_json, &serialized_msg->get_rcl_serialized_message());
      std::cout << "JSON: " << ss_json.str() << std::endl;
    }

    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    JsonEncoder json_encoder_;    
    std::shared_ptr<rclcpp::GenericSubscription> generic_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);


  rclcpp::spin(std::make_shared<RosbridgeCppNode>());
  rclcpp::shutdown();
  return 0;
}
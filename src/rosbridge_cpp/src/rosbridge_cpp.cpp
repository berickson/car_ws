#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2_introspection.hpp"

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

//      RCLCPP_INFO(this->get_logger(), "Got callback!");

      std::string message_identifier = "car/update";
      std::string message_type = "car_msgs/msg/Update";


      // register **once** your type

      static Ros2Introspection::FlatMessage flat_msg;
      unsigned max_array_size = 100;
      
      static bool first_time = true;
      if(first_time) {
        parser_.registerMessageType(message_identifier, message_type);
      }
      // parser_.deserializeIntoFlatMessage(message_identifier, &serialized_msg->get_rcl_serialized_message(), &flat_msg, max_array_size);  
      // // serialized_msg->release_rcl_serialized_message();

      

      // std::stringstream ss;
        
      // // you can show the entire content of the message as a list of key-values
      // for(const auto& pair: flat_msg.values)
      // {
      //   ss << pair.first.toStdString() << " = " << pair.second << std::endl;
      // }
      // for(const auto& pair: flat_msg.strings)
      // {
      //   ss << pair.first.toStdString() << " = " << pair.second << std::endl;
      // }
      // std::cout << ss.str() << std::endl;

      // RCLCPP_INFO(this->get_logger(), "Generic Message: \n'%s'", ss.str().c_str());

      std::stringstream ss_json;
      parser_.stream_json(message_identifier, ss_json, &serialized_msg->get_rcl_serialized_message());
      std::cout << "JSON: " << ss_json.str() << std::endl;
      

      
    }

    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    Ros2Introspection::Parser parser_;    
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
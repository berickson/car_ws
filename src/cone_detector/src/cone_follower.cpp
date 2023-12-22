#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "car_msgs/msg/rc_command.hpp"

float x_fov_degrees = 69;
int x_resolution = 640;


class cone_detector_node : public rclcpp::Node {
public:
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr subscription_;
  rclcpp::Publisher<car_msgs::msg::RcCommand>::SharedPtr rc_command_publisher_;

  void cone_detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr detections_msg) {
    for(auto detection : detections_msg->detections) {
      float x_angle_degrees = (x_resolution/2.0 - detection.bbox.center.position.x)/x_resolution * x_fov_degrees;
      RCLCPP_INFO(this->get_logger(), "detection degrees: %f", x_angle_degrees);

      car_msgs::msg::RcCommand rc_command_msg;
      rc_command_msg.str_us = 1500 - 200 * (x_angle_degrees/30) ;
      rc_command_msg.esc_us = 1500;
      rc_command_publisher_->publish(rc_command_msg);

      break; // we'll just grab the first detection (if any)
    }

  }


  cone_detector_node() : Node("cone_detector_node") {
    RCLCPP_INFO(this->get_logger(), "Cone Follower Node has started");
    subscription_ = this->create_subscription<vision_msgs::msg::Detection2DArray>("color/cone_detections", 1, std::bind(&cone_detector_node::cone_detection_callback, this, std::placeholders::_1));
    rc_command_publisher_ = this->create_publisher<car_msgs::msg::RcCommand>("car/rc_command", 1);
  }
  
};



int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

   auto node = std::make_shared<cone_detector_node>();

  rclcpp::spin(node);

  RCLCPP_INFO(node->get_logger(), "Cone Follower Node exiting");
  return 0;
}
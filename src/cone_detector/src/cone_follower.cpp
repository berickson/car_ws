#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

float x_fov_degrees = 69;
int x_resolution = 640;


class cone_follower_node : public rclcpp::Node {
public:
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

  void cone_detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr detections_msg) {
    for(auto detection : detections_msg->detections) {
      

      float x_angle_degrees = (x_resolution/2.0 - detection.bbox.center.position.x)/x_resolution * x_fov_degrees;
      float x_width_degrees = detection.bbox.size_x * x_fov_degrees / x_resolution;
      float width_radians = x_width_degrees * (M_PI / 180.0); 
      float cone_width = 0.3; // meters, width of the cone about camera height up
      float distance = (cone_width/2.0) / sin(width_radians/2.0);

      float max_velocity = 1.0;
      float max_acceleration = 0.5;
      float min_velocity = 0.1;
      float stop_distance = 0.35; // distance from cone to stop at
      float distance_remaining = distance - stop_distance;

      // velocity to stop in time
      float velocity = distance_remaining > 0 ? (2 * max_acceleration * distance_remaining) : 0;
      velocity = std::min(velocity, max_velocity);
      velocity = std::max(velocity, min_velocity);

      RCLCPP_INFO(this->get_logger(), "detection degrees: %3.1f width_degrees: %3.1f distance: %3.2f vel: %3.2f", x_angle_degrees, x_width_degrees, distance, velocity);

      geometry_msgs::msg::Twist cmd_vel_msg;
      cmd_vel_msg.linear.x = velocity;

      // turn angle_degrees in one meter
      float correction_distance = std::min(distance_remaining / 2, 1.0f); // distance to complete turn to currect angle
      cmd_vel_msg.angular.z = x_angle_degrees * (M_PI / 180.0) *  cmd_vel_msg.linear.x / correction_distance;

      // stop if too close
      if(distance < stop_distance) {
        cmd_vel_msg.linear.x = 0;
        cmd_vel_msg.angular.z = 0;
      }


      cmd_vel_publisher_->publish(cmd_vel_msg);

      break; // we'll just grab the first detection (if any)
    }
  }


  cone_follower_node() : Node("cone_follower_node") {
    RCLCPP_INFO(this->get_logger(), "Cone Follower Node has started");
    subscription_ = this->create_subscription<vision_msgs::msg::Detection2DArray>("car/oakd/color/cone_detections", 1, std::bind(&cone_follower_node::cone_detection_callback, this, std::placeholders::_1));
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  }
  
};



int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

   auto node = std::make_shared<cone_follower_node>();

  rclcpp::spin(node);

  RCLCPP_INFO(node->get_logger(), "Cone Follower Node exiting");
  return 0;
}
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "car_msgs/action/follow_cone.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::placeholders;


float x_fov_degrees = 69;
int x_resolution = 1200;

class cone_follower_node : public rclcpp::Node {
public:
  using FollowCone = car_msgs::action::FollowCone;
  using GoalHandleFollowCone = rclcpp_action::ServerGoalHandle<FollowCone>;

  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp_action::Server<FollowCone>::SharedPtr action_server_;


  std::shared_ptr<GoalHandleFollowCone> goal_handle_;

  void stop_car() {
    cmd_vel_publisher_->publish(geometry_msgs::msg::Twist());
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowCone::Goal> goal) 
  {
    if(goal_handle_.get() != nullptr) {
      RCLCPP_INFO(this->get_logger(), "Received new goal request, canceling previous goal");
      auto result = std::make_shared<FollowCone::Result>();
      goal_handle_->abort(result);
      goal_handle_ = nullptr;
    } 
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFollowCone> goal_handle)
  {
    goal_handle_ = goal_handle;
    // this is message based, so we don't need to do anything here, we can create a thread
    // if we want to do something in the background
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFollowCone> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    goal_handle_ = nullptr;
    return rclcpp_action::CancelResponse::ACCEPT;
  }



  void cone_detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr detections_msg) {
    if(goal_handle_.get() == nullptr) {
      return;
    }
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
      float velocity = distance_remaining > 0 ?
        std::clamp(2 * max_acceleration * distance_remaining, min_velocity, max_velocity) 
        : 0;

      RCLCPP_INFO(this->get_logger(), "detection degrees: %3.1f width_degrees: %3.1f distance: %3.2f vel: %3.2f", x_angle_degrees, x_width_degrees, distance, velocity);

      geometry_msgs::msg::Twist cmd_vel_msg;
      cmd_vel_msg.linear.x = velocity;
      if(velocity > 0) {
        float correction_distance = std::clamp(distance_remaining / 2, 0.5f, 2.0f); // distance to complete turn to currect angle
        cmd_vel_msg.angular.z = x_angle_degrees * (M_PI / 180.0) *  cmd_vel_msg.linear.x / correction_distance;
      } else {
        cmd_vel_msg.angular.z = 0;
      }

      cmd_vel_publisher_->publish(cmd_vel_msg);
      if(distance_remaining <= 0) {
        if(goal_handle_.get() != nullptr) {
          RCLCPP_INFO(this->get_logger(), "Goal reached, completing action");
          goal_handle_->succeed(std::make_shared<FollowCone::Result>());
          goal_handle_ = nullptr;
        }
      }

      break; // we'll just grab the first detection (if any)
    }
  }


  cone_follower_node() : Node("cone_follower_node") {
    RCLCPP_INFO(this->get_logger(), "Cone Follower Node has started");
    subscription_ = this->create_subscription<vision_msgs::msg::Detection2DArray>("car/oakd/color/cone_detections", 1, std::bind(&cone_follower_node::cone_detection_callback, this, std::placeholders::_1));
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

    this->action_server_ = rclcpp_action::create_server<FollowCone>(
      this,
      "follow_cone",
      std::bind(&cone_follower_node::handle_goal, this, _1, _2),
      std::bind(&cone_follower_node::handle_cancel, this, _1),
      std::bind(&cone_follower_node::handle_accepted, this, _1));
    }
  
};



int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

   auto node = std::make_shared<cone_follower_node>();

  rclcpp::spin(node);

  RCLCPP_INFO(node->get_logger(), "Cone Follower Node exiting");
  return 0;
}
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "car_msgs/action/follow_cone.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <limits>

using namespace std::placeholders;


float x_fov_degrees = 1.1 * 69;
int x_resolution = 320;

class cone_follower_node : public rclcpp::Node {
public:
  using FollowCone = car_msgs::action::FollowCone;
  using GoalHandleFollowCone = rclcpp_action::ServerGoalHandle<FollowCone>;

  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_subscription_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_subscription_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp_action::Server<FollowCone>::SharedPtr action_server_;
  
  // save the most recent marker
  visualization_msgs::msg::Marker::SharedPtr goal_marker_;


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

  void cone_marker_callback(const visualization_msgs::msg::MarkerArray::SharedPtr markers_msg) {
    if(markers_msg->markers.size() < 2) {
      return;
    }
    goal_marker_ = std::make_shared<visualization_msgs::msg::Marker>(markers_msg->markers[1]);
  }

  void loop() {
    if(goal_handle_.get() == nullptr) {
      return;
    }
 
    // fail if we don't have messages
    if(goal_marker_ == nullptr) {
      RCLCPP_WARN(this->get_logger(), "No valid marker message");
      goal_handle_->abort(std::make_shared<FollowCone::Result>());
      goal_handle_ = nullptr;
      return;
    }

    // calculate difference between current time and time of detection
    auto marker_time_diff = this->now() - goal_marker_->header.stamp;
    if(marker_time_diff > rclcpp::Duration(std::chrono::seconds(1))) {
      goal_handle_->abort(std::make_shared<FollowCone::Result>());
      goal_handle_ = nullptr;
      RCLCPP_INFO(this->get_logger(), "Marker message is too old");
      return;
    }

    geometry_msgs::msg::PointStamped p1,p2;
    auto & marker = *goal_marker_;

    // find transform from where the bumber is now to where the cone was when it was detected
    rclcpp::Time now = this->get_clock()->now();
    rclcpp::Time when = marker.header.stamp;
    auto to_frame = "front_bumper";
    auto from_frame = marker.header.frame_id;
    auto fixed_frame = "odom";
    auto t = tf_buffer_->lookupTransform(
        to_frame,
        now,
        from_frame,
        when,
        fixed_frame,
        std::chrono::milliseconds(50)
    );

    p1.header = marker.header;
    p1.point = marker.pose.position;

    tf2::doTransform(p1, p2, t);

    RCLCPP_INFO(this->get_logger(), "Marker: x: %5.2f y: %5.2f z: %5.2f", p2.point.x, p2.point.y, p2.point.z);

    if(goal_handle_.get() == nullptr) {
      return;
    }

    float distance = p2.point.x;
    float cone_angle = atan2(p2.point.y, p2.point.x);

    double max_velocity;
    get_parameter<double>("max_velocity", max_velocity);
    double max_accel;
    get_parameter<double>("accel", max_accel);
    double min_velocity;
    get_parameter<double>("min_velocity", min_velocity);
    double stop_distance; // distance from front of car to cone to stop at
    get_parameter<double>("goal_distance", stop_distance);
    double distance_remaining = distance - stop_distance;

    double lag_time = 0.3; // expected system lag in velocity PID
    // velocity to stop in time
    float velocity = distance_remaining > 0 ?
      std::clamp(sqrt(2 * max_accel * distance_remaining)-lag_time*max_accel, min_velocity, max_velocity) 
      : 0;

    geometry_msgs::msg::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = velocity;
    if(velocity > 0) {
      float correction_distance = std::clamp(distance_remaining / 2, 0.5, 2.0); // distance to complete turn to currect angle
      cmd_vel_msg.angular.z = cone_angle *  cmd_vel_msg.linear.x / correction_distance;
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
  }

  // timer member variable
  rclcpp::TimerBase::SharedPtr timer_;

  cone_follower_node() : Node("cone_follower_node") {
    RCLCPP_INFO(this->get_logger(), "Cone Follower Node has started");
    marker_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>("car/oakd/color/cone_markers", 1, std::bind(&cone_follower_node::cone_marker_callback, this, std::placeholders::_1));

    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

    this->action_server_ = rclcpp_action::create_server<FollowCone>(
      this,
      "follow_cone",
      std::bind(&cone_follower_node::handle_goal, this, _1, _2),
      std::bind(&cone_follower_node::handle_cancel, this, _1),
      std::bind(&cone_follower_node::handle_accepted, this, _1));

      {
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Max acceleration";
        param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        this->declare_parameter("accel", 0.3, param_desc);
      }

      {
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Max velocity";
        param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        this->declare_parameter("max_velocity",1.5, param_desc);
      }

      {
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Goal distance to cone from front of car";
        param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        this->declare_parameter("goal_distance", 0.4, param_desc);
      }

      {
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Min velocity to command since zero velocity can cause issues";
        param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        this->declare_parameter("min_velocity",0.1, param_desc);
      }

      // loop at 100Hz
      timer_ = this->create_wall_timer(std::chrono::milliseconds(10), [this]() {
        loop();
      });

    }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<cone_follower_node>();

  rclcpp::spin(node);

  RCLCPP_INFO(node->get_logger(), "Cone Follower Node exiting");
  return 0;
}
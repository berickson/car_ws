#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"

#include "car_msgs/action/rotate_in_place.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::placeholders;


class rotate_in_place_node : public rclcpp::Node {
public:
  using RotateInPlace = car_msgs::action::RotateInPlace;
  using GoalHandleRotateInPlace = rclcpp_action::ServerGoalHandle<RotateInPlace>;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp_action::Server<RotateInPlace>::SharedPtr action_server_;


  std::shared_ptr<GoalHandleRotateInPlace> goal_handle_;

  void stop_car() {
    cmd_vel_publisher_->publish(geometry_msgs::msg::Twist());
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const RotateInPlace::Goal> goal) 
  {
    if(goal_handle_.get() != nullptr) {
      RCLCPP_INFO(this->get_logger(), "Received new goal request, canceling previous goal");
      auto result = std::make_shared<RotateInPlace::Result>();
      goal_handle_->abort(result);
      goal_handle_ = nullptr;
    } 
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleRotateInPlace> goal_handle)
  {
    goal_handle_ = goal_handle;
    // this is message based, so we don't need to do anything here, we can create a thread
    // if we want to do something in the background
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleRotateInPlace> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    goal_handle_ = nullptr;
    return rclcpp_action::CancelResponse::ACCEPT;
  }



  rotate_in_place_node() : Node("rotate_in_place_node") {
    RCLCPP_INFO(this->get_logger(), "Rotate in Place Node has started");
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

    this->action_server_ = rclcpp_action::create_server<RotateInPlace>(
      this,
      "rotate_in_place",
      std::bind(&rotate_in_place_node::handle_goal, this, _1, _2),
      std::bind(&rotate_in_place_node::handle_cancel, this, _1),
      std::bind(&rotate_in_place_node::handle_accepted, this, _1));
    }
};



#ifndef CONE_FOLLOWER_CLIENT_HPP_
#define CONE_FOLLOWER_CLIENT_HPP_

#include "car_msgs/action/follow_cone.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::placeholders;
// see 
// https://docs.ros.org/en/iron/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html

class ConeFollowerClient : public rclcpp::Node {
  public:
  using FollowCone = car_msgs::action::FollowCone;
  using GoalHandleFollowCone = rclcpp_action::ClientGoalHandle<FollowCone>;

  rclcpp_action::Client<FollowCone>::SharedPtr client_ptr_;

  ConeFollowerClient() : Node("cone_follower_client") {
    //log
    RCLCPP_INFO(this->get_logger(), "ConeFollowerClient node started");
  }

  void send_goal() {
    RCLCPP_INFO(this->get_logger(), "Sending goal");
    this->client_ptr_ = rclcpp_action::create_client<FollowCone>(
      this,
      "follow_cone");

    // wait for up to 10 seconds for the action server to become available
    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = FollowCone::Goal();

    auto send_goal_options = rclcpp_action::Client<FollowCone>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&ConeFollowerClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&ConeFollowerClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&ConeFollowerClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(this->get_logger(), "Goal sent");
  };

  void goal_response_callback(const GoalHandleFollowCone::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

    void feedback_callback(
    GoalHandleFollowCone::SharedPtr,
    const std::shared_ptr<const FollowCone::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Goal feedback received: ";
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandleFollowCone::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Result received: ";
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
};

#endif  // CONE_FOLLOWER_CLIENT_HPP_

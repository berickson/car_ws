
#include "cone_follower_client.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <string>


void goto_pose(std::string frame, float x, float y, float degrees) {
  float theta = degrees * M_PI / 180.0;
  // use nav2 to go to a pose
  auto node = std::make_shared<rclcpp::Node>("goto_pose");
  auto client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node, "/navigate_to_pose");

  // wait for up to 10 seconds for the action server to become available
  if (!client->wait_for_action_server(std::chrono::seconds(3))) {
    RCLCPP_ERROR(rclcpp::get_logger("goto_pose"), "/navigate_to_pose Action server not available after waiting");
    rclcpp::shutdown();
  }
  
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.header.frame_id = frame;
  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;
  goal.pose.pose.position.z = 0.0;
  goal.pose.pose.orientation.x = 0.0;
  goal.pose.pose.orientation.y = 0.0;
  goal.pose.pose.orientation.z = sin(theta/2);
  goal.pose.pose.orientation.w = cos(theta/2);
  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [](std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(rclcpp::get_logger("goto_pose"), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(rclcpp::get_logger("goto_pose"), "Goal accepted by server, waiting for result");
      }
    };

  bool done = false;
  send_goal_options.result_callback =
    [&done](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
      done = true;
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(rclcpp::get_logger("goto_pose"), "Goal succeeded");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_INFO(rclcpp::get_logger("goto_pose"), "Goal was aborted");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_INFO(rclcpp::get_logger("goto_pose"), "Goal was canceled");
          return;
        default:
          RCLCPP_INFO(rclcpp::get_logger("goto_pose"), "Unknown result code");
          return;

      }
    };
  auto future = client->async_send_goal(goal, send_goal_options);
  RCLCPP_INFO(rclcpp::get_logger("goto_pose"), "Goal sent");
  while(!done) {
    rclcpp::spin_some(node);
  } 
  RCLCPP_INFO(rclcpp::get_logger("goto_pose"), "goto_pose complete");
}
    

void goto_cone() {

  // create ConeFollowerClient node
  auto node = std::make_shared<ConeFollowerClient>();
  auto future = node->send_goal();
  while(!node->is_goal_done) {
    rclcpp::spin_some(node);
  }
  RCLCPP_INFO(node->get_logger(), "goto_cone complete");
}

// back up the car
// example from terminal: 
// ros2 action send_goal /backup nav2_msgs/action/BackUp "{target: {x: -1}, speed: 0.5, time_allowance: {sec: 5}}"
void back_up(float meters, float speed = 0.5, float time_allowance = 5.0) {
  // use nav2 to back up
  auto node = std::make_shared<rclcpp::Node>("back_up");
  auto client = rclcpp_action::create_client<nav2_msgs::action::BackUp>(node, "/backup");

  // wait for up to 10 seconds for the action server to become available
  if (!client->wait_for_action_server(std::chrono::seconds(3))) {
    RCLCPP_ERROR(rclcpp::get_logger("back_up"), "/backup Action server not available after waiting");
    rclcpp::shutdown();
  }
  
  auto goal = nav2_msgs::action::BackUp::Goal();
  goal.target.x = 1.0 * fabs(meters);
  goal.speed = speed;
  goal.time_allowance.sec = time_allowance;
  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::BackUp>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [](std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::BackUp>> goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(rclcpp::get_logger("back_up"), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(rclcpp::get_logger("back_up"), "Goal accepted by server, waiting for result");
      }
    };

  bool done = false;
  send_goal_options.result_callback =
    [&done](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::BackUp>::WrappedResult & result) {
      done = true;
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(rclcpp::get_logger("back_up"), "Goal succeeded");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_INFO(rclcpp::get_logger("back_up"), "Goal was aborted");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_INFO(rclcpp::get_logger("back_up"), "Goal was canceled");
          return;
        default:
          RCLCPP_INFO(rclcpp::get_logger("back_up"), "Unknown result code");
          return;
      }
    };
  auto future = client->async_send_goal(goal, send_goal_options);
  RCLCPP_INFO(rclcpp::get_logger("back_up"), "Goal sent");
  RCLCPP_INFO(rclcpp::get_logger("back_up"), "back_up complete");
  while(!done) {
    rclcpp::spin_some(node);
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  //goto_cone();
  //back_up(1.0);
  //goto_pose("base_link", 1.5, 1.5, 90);
  goto_cone();
  //goto_pose("base_link", 0.0, 0.0, 200);
  //goto_cone();
  return 0;
  /*
  // create ConeFollowerClient node
  auto node = std::make_shared<ConeFollowerClient>();
  node->send_goal();
  rclcpp::spin(node);
  return 0;
  */
}
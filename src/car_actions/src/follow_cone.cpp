#include "cone_follower_client.hpp"
#include <csignal>

std::shared_ptr<ConeFollowerClient> node;

bool cancel = false;

// enable cancelling with ctrl-c, etc.
void signalHandler(int signum) {
  if (node) {
    node->cancel_goal();
  }
  cancel = true;
}


int main(int argc, char** argv) {
  // create ConeFollowerClient node
  rclcpp::init(argc, argv);
  node = std::make_shared<ConeFollowerClient>();
  std::signal(SIGINT, signalHandler);
  node->send_goal();
  while(!node->is_goal_done && !cancel) {
    rclcpp::spin_some(node);
  }
  node = nullptr;
  return 0;
}
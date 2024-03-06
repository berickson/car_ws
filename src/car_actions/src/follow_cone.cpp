#include "cone_follower_client.hpp"

int main(int argc, char** argv) {
  // create ConeFollowerClient node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ConeFollowerClient>();
  node->send_goal();
  rclcpp::spin(node);
  return 0;
}

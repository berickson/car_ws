#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "car_msgs/msg/update.hpp"
#include "car_msgs/msg/rc_command.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "car_msgs/msg/speedometer.hpp"
#include "geometry.h"
#include "lookup_table.h"
#include "pid.h"


using std::placeholders::_1;




int steering_for_curvature(Angle theta_per_meter) {
  static const LookupTable t({{-85.1, 1929},
                              {-71.9, 1839},
                              {-58.2, 1794},
                              {-44.1, 1759},
                              {-29.6, 1678},
                              {-14.8, 1599},
                              {0, 1521},
                              {14.8, 1461},
                              {29.6, 1339},
                              {44.0, 1306},
                              {58.2, 1260},
                              {71.9, 1175},
                              {85.1, 1071}

  });
  return (int)t.lookup(theta_per_meter.degrees());
}

int esc_for_velocity(double v) {
  static const LookupTable t({
      {-2., 1300},  // {-2., 1200},
      {-1., 1400},  // {-1., 1250},
      {-.1, 1450},  // {-.1, 1326},
      {0.0, 1500},  // {0.0,  1500},
      {0.1, 1550},  // {0.1,  1610},
      {0.5, 1560},  // {0.5, 1620},
      {2.0, 1570},  // {2.0, 1671},
      {3.3, 1630},  // {3.3, 1700},
      {4.1, 1680},  // {4.1, 1744},
      {5, 1710},    // {5, 1770},
      {7, 1827},    // {7, 1887},
      {9.5, 1895},  // {9.5,1895},
      {20, 2000}    // {20, 2000}
  });
  return t.lookup(v);
}

class CarControllerNode : public rclcpp::Node
{
  public:

    PID velocity_pid;

    CarControllerNode()
    : Node("car_controller")
    {

      rc_command_publisher_ = this->create_publisher<car_msgs::msg::RcCommand>("car/rc_command", 1);

      car_update_subscription_ = this->create_subscription<car_msgs::msg::Update>(
      "car/update", rclcpp::SensorDataQoS(), std::bind(&CarControllerNode::car_update_topic_callback, this, _1));

      cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 1, std::bind(&CarControllerNode::cmd_vel_topic_callback, this, _1));

      motor_speedometer_subscription_ = this->create_subscription<car_msgs::msg::Speedometer>(
      "/car/speedometers/motor", 1, std::bind(&CarControllerNode::motor_speedometer_topic_callback, this, _1));

      velocity_pid.k_p = 1.0;
      velocity_pid.k_i = 0.0;
      velocity_pid.k_d = 10.0;

    }

  private:

    void motor_speedometer_topic_callback(car_msgs::msg::Speedometer::SharedPtr msg)
    {
      RCLCPP_INFO_ONCE(this->get_logger(), "Receiving speedometer messages");
      speedometer_message = msg;
    }


    void cmd_vel_topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
    {
      RCLCPP_INFO_ONCE(this->get_logger(), "Receiving cmd_vel messages");
      // sanity checks
      if(
        msg->linear.y != 0.0 
        || msg->linear.z != 0.0
        || msg->angular.x != 0.0
        || msg->angular.y != 0.0
      ) {
        RCLCPP_WARN_ONCE (this->get_logger(), "Ingoring invalid cmd_vel, must only have linear.x and angular.z componets");
        return;
      }

      double theta_per_second = msg->angular.z;
      double meters_per_second = msg->linear.x;
      Angle theta_per_meter = Angle::radians( meters_per_second == 0.0 ? 0.0 : theta_per_second / meters_per_second );

      car_msgs::msg::RcCommand rc_command_message;
      rc_command_message.str_us = steering_for_curvature(theta_per_meter);

      double v_error = 0;

      if(speedometer_message) {
        rclcpp::Time t2 = this->now();
        rclcpp::Time t1(speedometer_message->header.stamp);
        auto delta = t2-t1;
        if(delta < rclcpp::Duration(0,300000000)) {
          RCLCPP_INFO(this->get_logger(), "times align");
        }
        v_error = meters_per_second - speedometer_message->v_smooth;
        rc_command_message.esc_us = esc_for_velocity(meters_per_second + velocity_pid.k_p * v_error);
        rc_command_publisher_->publish(rc_command_message);
      }
    }

    void car_update_topic_callback(const car_msgs::msg::Update::SharedPtr msg) const
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "Running and receiving car update messages");
    }


    car_msgs::msg::Speedometer::SharedPtr speedometer_message;

    rclcpp::Subscription<car_msgs::msg::Update>::SharedPtr car_update_subscription_;
    rclcpp::Subscription<car_msgs::msg::Speedometer>::SharedPtr motor_speedometer_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Publisher<car_msgs::msg::RcCommand>::SharedPtr rc_command_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CarControllerNode>());
  rclcpp::shutdown();
  return 0;
}
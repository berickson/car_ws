#include <memory>

#include "rclcpp/rclcpp.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/empty.hpp"

#include "car_msgs/msg/update.hpp"
#include "car_msgs/msg/rc_command.hpp"
#include "car_msgs/msg/speedometer.hpp"

#include "speedometer.h"
#include "ackermann.h"
#include "geometry.h"
#include "lookup_table.h"
#include "pid.h"


using std::placeholders::_1;
using std::placeholders::_2;


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
    void update_callback(const car_msgs::msg::Update::SharedPtr update);

    Speedometer front_right_wheel_;
    Speedometer front_left_wheel_;
    Speedometer motor_;

    CarControllerNode()
    : Node("car_controller")
    {

      {
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Velocity PID k_p term";
        param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        
        rcl_interfaces::msg::FloatingPointRange range;
        range.set__from_value(0.0).set__to_value(10.0);

        param_desc.floating_point_range= {range};

        this->declare_parameter("velocity_k_p", 2.0, param_desc);
      }


      {
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Velocity PID k_a term";
        param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        
        rcl_interfaces::msg::FloatingPointRange range;
        range.set__from_value(0.0).set__to_value(10.0);

        param_desc.floating_point_range= {range};

        this->declare_parameter("velocity_k_a", 0.3, param_desc);
      }

      front_right_wheel_.meters_per_tick = front_meters_per_odometer_tick;
      front_left_wheel_.meters_per_tick = front_meters_per_odometer_tick;
      motor_.meters_per_tick = motor_meters_per_odometer_tick;

      fl_speedometer_publisher_ = this->create_publisher<car_msgs::msg::Speedometer> ("/car/speedometers/fl", 10);
      fr_speedometer_publisher_ = this->create_publisher<car_msgs::msg::Speedometer> ("/car/speedometers/fr", 10);
      motor_speedometer_publisher_ = this->create_publisher<car_msgs::msg::Speedometer> ("/car/speedometers/motor", 10);
      ackerman_fr_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped> ("/car/ackermann/fr", 10);
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      rc_command_publisher_ = this->create_publisher<car_msgs::msg::RcCommand>("car/rc_command", 1);

      car_update_subscription_ = this->create_subscription<car_msgs::msg::Update>(
      "car/update", rclcpp::SensorDataQoS(), std::bind(&CarControllerNode::car_update_topic_callback, this, _1));

      cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 1, std::bind(&CarControllerNode::cmd_vel_topic_callback, this, _1));

      reset_service_ = this->create_service<std_srvs::srv::Empty>("car/reset",std::bind(&CarControllerNode::reset_service_callback, this, _1, _2) );
      

      velocity_pid.k_p = 1.0;
      velocity_pid.k_i = 0.0;
      velocity_pid.k_d = 10.0;

    }

  private:
    float esc_us_float = 1500;
    float str_us_float = 1500;

    int steering_for_angle(Angle theta);
    int steering_for_curvature(Angle theta_per_meter) const;
    Angle angle_for_steering(int str);
    void car_update_topic_callback(const car_msgs::msg::Update::SharedPtr d);

    int32_t update_count_ = 0;

    // car constants for blue-crash
    // todo: get from parameter server
    const float front_meters_per_odometer_tick = 0.002528;
    const float rear_meters_per_odometer_tick = 0.00146;
    const float motor_meters_per_odometer_tick = 0.00292;
    const float front_wheelbase_width_in_meters = 0.2413;
    const float rear_wheelbase_width_in_meters = 0.2667;
    const float wheelbase_length_in_meters = 0.33655;

    rclcpp::Publisher<car_msgs::msg::Speedometer>::SharedPtr fl_speedometer_publisher_;
    rclcpp::Publisher<car_msgs::msg::Speedometer>::SharedPtr fr_speedometer_publisher_;
    rclcpp::Publisher<car_msgs::msg::Speedometer>::SharedPtr motor_speedometer_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ackerman_fr_publisher_;

    rclcpp::Subscription<car_msgs::msg::Update>::SharedPtr update_sub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    geometry_msgs::msg::Twist::SharedPtr cmd_vel_message;
    rclcpp::Time cmd_vel_receive_time;


    Ackermann ackermann_;
    car_msgs::msg::Speedometer last_fr_;
    car_msgs::msg::Speedometer last_fl_;
    car_msgs::msg::Speedometer last_motor_;


    void cmd_vel_topic_callback(geometry_msgs::msg::Twist::SharedPtr msg)
    {
      RCLCPP_INFO_ONCE(this->get_logger(), "Receiving cmd_vel messages");
      cmd_vel_message = msg;
      cmd_vel_receive_time = now();

    }

    void reset_service_callback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Empty::Response>  /*response*/)
    {
      RCLCPP_INFO(this->get_logger(), "reset");
      ackermann_.reset();
    }

    car_msgs::msg::Speedometer::SharedPtr speedometer_message;

    rclcpp::Subscription<car_msgs::msg::Update>::SharedPtr car_update_subscription_;
    rclcpp::Subscription<car_msgs::msg::Speedometer>::SharedPtr motor_speedometer_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;

    rclcpp::Publisher<car_msgs::msg::RcCommand>::SharedPtr rc_command_publisher_;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
};

int CarControllerNode::steering_for_curvature(Angle theta_per_meter) const {
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

int CarControllerNode::steering_for_angle(Angle theta) {
  static const LookupTable t({{-30, 1929},
                              {-25, 1839},
                              {-20, 1794},
                              {-15, 1759},
                              {-10, 1678},
                              {-5, 1599},
                              {0, 1521},
                              {5, 1461},
                              {10, 1339},
                              {15, 1306},
                              {20, 1260},
                              {25, 1175},
                              {30, 1071}

  });
  return (int)t.lookup(theta.degrees());
}

Angle CarControllerNode::angle_for_steering(int str) {
  static const LookupTable t({{1071, 30},
                              {1175, 25},
                              {1260, 20},
                              {1306, 15},
                              {1339, 10},
                              {1461, 5},
                              {1521, 0},
                              {1599, -5},
                              {1678, -10},
                              {1759, -15},
                              {1794, -20},
                              {1839, -25},
                              {1929, -30}});

  return Angle::degrees(t.lookup(str));
}

void CarControllerNode::car_update_topic_callback(const car_msgs::msg::Update::SharedPtr d){
    RCLCPP_INFO_ONCE(this->get_logger(), "Running and receiving car update messages");

    ++update_count_;

    Angle yaw = Angle::degrees(d->mpu_deg_yaw);
    Angle pitch = Angle::degrees(d->mpu_deg_pitch);
    Angle roll = Angle::degrees(d->mpu_deg_roll);

    front_left_wheel_.update_from_sensor(d->us, d->odo_fl_a, d->odo_fl_a_us,
                                        d->odo_fl_b, d->odo_fl_b_us);
    motor_.update_from_sensor(d->us, d->spur_odo, d->spur_us);
    front_right_wheel_.update_from_sensor(d->us, d->odo_fr_a, d->odo_fr_a_us, 
                                         d->odo_fr_b, d->odo_fr_b_us);

    auto fl = front_left_wheel_.get_speedometer_message();
    fl.header = d->header;
    fl_speedometer_publisher_->publish(fl);

    auto fr = front_right_wheel_.get_speedometer_message();
    fr.header = d->header;
    fr_speedometer_publisher_->publish(fr);

    auto motor = motor_.get_speedometer_message();
    motor.header = d->header;
    motor_speedometer_publisher_->publish(motor);

    if(update_count_==1) {
        ackermann_ = Ackermann(front_wheelbase_width_in_meters, wheelbase_length_in_meters, Point(0, 0),
                            yaw);  // ackermann needs first heading reading
    } else if (update_count_ > 2) {
        double wheel_distance_meters = fr.meters - last_fr_.meters;
        if(fabs(wheel_distance_meters)>0) {
            Angle outside_wheel_angle = angle_for_steering(d->rx_str);
            ackermann_.move_right_wheel(outside_wheel_angle, wheel_distance_meters,
                                    yaw);
        } 
    }
    
    geometry_msgs::msg::PoseStamped pose_msg;

    Point rear_position = ackermann_.rear_position();
    
    pose_msg.header.stamp = d->header.stamp;
    pose_msg.header.frame_id = "odom";
    // pose_msg.child_frame_id = turtle_name;
    pose_msg.pose.position.x = rear_position.x;
    pose_msg.pose.position.y = rear_position.y;
    pose_msg.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(roll.radians(), -pitch.radians(), yaw.radians());
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    ackerman_fr_publisher_->publish(pose_msg);


    geometry_msgs::msg::TransformStamped tf_msg;

    // tf_msg.header.stamp = d->header.stamp;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_footprint";
    tf_msg.header.stamp = d->header.stamp;
    tf_msg.transform.translation.x = rear_position.x;
    tf_msg.transform.translation.y = rear_position.y;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf_msg);

    last_fr_ = fr;
    last_fl_ = fl;
    last_motor_ = motor;

    if(cmd_vel_message && (now()-cmd_vel_receive_time) < 500ms ) {

      double theta_per_second = cmd_vel_message->angular.z;
      double meters_per_second = cmd_vel_message->linear.x;
      Angle theta_per_meter = Angle::radians( meters_per_second == 0.0 ? 0.0 : theta_per_second / meters_per_second );
      str_us_float = steering_for_curvature(theta_per_meter);

      if(meters_per_second > 0 && esc_us_float < 1540) {
        esc_us_float = 1540;
      } else if (meters_per_second < 0 && esc_us_float > 1460) {
        esc_us_float = 1460;
      }

      double velocity_k_p, velocity_k_a;
      get_parameter("velocity_k_p", velocity_k_p);
      get_parameter("velocity_k_a", velocity_k_a);
      double v_error = meters_per_second - motor.v_smooth;
      double a_error = -motor.a_smooth;
      esc_us_float += velocity_k_p * v_error + velocity_k_a * a_error ;

      // handle stopped case
      if(fabs(meters_per_second)<0.02 && fabs(motor.v_smooth)<0.02) {
        esc_us_float = 1500;
        str_us_float = 1500;
      }
    } else {
      esc_us_float = 1500; // idle
      str_us_float = 1500; // idle
    }

    car_msgs::msg::RcCommand rc_command_message;
    rc_command_message.str_us = str_us_float;
    rc_command_message.esc_us = esc_us_float;
    rc_command_publisher_->publish(rc_command_message);

}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CarControllerNode>());
  rclcpp::shutdown();
  return 0;
}
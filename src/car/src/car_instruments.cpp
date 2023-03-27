#include "rclcpp/rclcpp.hpp"
#include <memory>

#include "car_msgs/msg/update.hpp"
#include "car_msgs/msg/speedometer.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "speedometer.h"
#include "ackermann.h"
#include "lookup_table.h"
using std::placeholders::_1;

class CarInstrumentsNode : public rclcpp::Node {
     public:
        CarInstrumentsNode();
        void update_callback(const car_msgs::msg::Update::SharedPtr update);

        Speedometer front_right_wheel_;
        Speedometer front_left_wheel_;
        Speedometer motor_;

     private:
        int steering_for_angle(Angle theta);
        int steering_for_curvature(Angle theta_per_meter);
        Angle angle_for_steering(int str);

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


        Ackermann ackermann_;
        car_msgs::msg::Speedometer last_fr_;
        car_msgs::msg::Speedometer last_fl_;
        car_msgs::msg::Speedometer last_motor_;

};

CarInstrumentsNode::CarInstrumentsNode() : Node("car_instruments") {
    front_right_wheel_.meters_per_tick = front_meters_per_odometer_tick;
    front_left_wheel_.meters_per_tick = front_meters_per_odometer_tick;
    motor_.meters_per_tick = motor_meters_per_odometer_tick;

    fl_speedometer_publisher_ = this->create_publisher<car_msgs::msg::Speedometer> ("/car/speedometers/fl", 10);
    fr_speedometer_publisher_ = this->create_publisher<car_msgs::msg::Speedometer> ("/car/speedometers/fr", 10);
    motor_speedometer_publisher_ = this->create_publisher<car_msgs::msg::Speedometer> ("/car/speedometers/motor", 10);
    ackerman_fr_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped> ("/car/ackermann/fr", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    update_sub_ = this->create_subscription<car_msgs::msg::Update> ("/car/update", rclcpp::SensorDataQoS(), std::bind(&CarInstrumentsNode::update_callback, this, _1));

}

int CarInstrumentsNode::steering_for_curvature(Angle theta_per_meter) {
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

int CarInstrumentsNode::steering_for_angle(Angle theta) {
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

Angle CarInstrumentsNode::angle_for_steering(int str) {
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

void CarInstrumentsNode::update_callback(const car_msgs::msg::Update::SharedPtr d){
    // ROS_INFO("got car update ms: %d us: %d", d->ms, d->us);
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
    // fl.header = d->header;
    fl_speedometer_publisher_->publish(fl);

    auto fr = front_right_wheel_.get_speedometer_message();
    // fr.header = d->header;
    fr_speedometer_publisher_->publish(front_right_wheel_.get_speedometer_message());

    auto motor = motor_.get_speedometer_message();
    // motor.header = d->header;
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

            // br.sendTransform(transformStamped);
                        

        } 
    }
    
    geometry_msgs::msg::PoseStamped pose_msg;

    Point rear_position = ackermann_.rear_position();
    
    //pose_msg.header.stamp = d->header.stamp;
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
    tf_msg.child_frame_id = "base_link";
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
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CarInstrumentsNode>());
  rclcpp::shutdown();
  return 0;
}
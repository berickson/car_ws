#include <memory>
#include<unistd.h>

#include "rclcpp/rclcpp.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "nav_msgs/msg/odometry.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "sensor_msgs/msg/imu.hpp"

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

// helper, maps a value from one range [a1,a2] to [b1,b2]
double mapRange(double a1, double a2, double b1, double b2, double s)
{
    return b1 + ((s - a1) * (b2 - b1)) / (a2 - a1);
}

class Car : public rclcpp::Node
{
  public:

    PID velocity_pid;
    void update_callback(const car_msgs::msg::Update::SharedPtr update);

    Speedometer front_right_wheel_;
    Speedometer front_left_wheel_;
    Speedometer motor_;

    Car()
    : Node("car")
    {


      {
        // robot_id
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Unique name of this robot";
        param_desc.type = rclcpp::ParameterType::PARAMETER_STRING;
        this->declare_parameter("robot_id", "unknown", param_desc);
      }


      {
        // mag_x_min
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Minimum raw value of x from magnetometer";
        param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        this->declare_parameter("mag_x_min", -1000.0, param_desc);
      }

      {
        // mag_x_max
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Maximum raw value of x from magnetometer";
        param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        this->declare_parameter("mag_x_max", 1000.0, param_desc);
      }

      {
        // mag_y_min
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Minimum raw value of y from magnetometer";
        param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        this->declare_parameter("mag_y_min", -1000.0, param_desc);
      }

      {
        // mag_y_max
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Maximum raw value of y from magnetometer";
        param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        this->declare_parameter("mag_y_max", 1000.0, param_desc);
      }



      {
        std::vector<double> default_velocity_to_esc_lookup          {
            -2., 1300,  // {-2., 1200},
            -1., 1400,  // {-1., 1250},
            -.1, 1450,  // {-.1, 1326},
            0.0, 1500,  // {0.0,  1500},
            0.1, 1550,  // {0.1,  1610},
            0.5, 1560,  // {0.5, 1620},
            2.0, 1570,  // {2.0, 1671},
            3.3, 1630,  // {3.3, 1700},
            4.1, 1680,  // {4.1, 1744},
            5, 1710,    // {5, 1770},
            7, 1827,    // {7, 1887},
            9.5, 1895,  // {9.5,1895},
            20, 2000    // {20, 2000} 
          };

        // esc_for_velocity lookup table
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Lookup table for ESC signal for velocity [{v1,esc1},{v2,esc2},...]";  
        this->declare_parameter<std::vector<double>>(
          "velocity_to_esc_lookup",
          default_velocity_to_esc_lookup, 
          param_desc);
      }
      {
        // curvature_to_str_lookup_table
        std::vector<double> default_curvature_to_str_lookup {
          -85.1, 1929.0,
          -71.9, 1839.0,
          -58.2, 1794.0,
          -44.1, 1759.0,
          -29.6, 1678.0,
          -14.8, 1599.0,
          0.0, 1521.0,
          14.8, 1461.0,
          29.6, 1339.0,
          44.0, 1306.0,
          58.2, 1260.0,
          71.9, 1175.0,
          85.1, 1071.0 
        };

        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Lookup table for steering signal for curvature [{curvature1,str1},{curvature2,str2},...]";
        this->declare_parameter<std::vector<double>>(
          "curvature_to_str_lookup",
          default_curvature_to_str_lookup, 
          param_desc);
      }

      {
        // compass_correction_degrees
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Calibration constant will be added to the yaw reading from the compass";
        param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        this->declare_parameter("compass_correction_degrees", 0.0, param_desc);
      }

      // compass error degrees
      {
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Standard deviation for compass reading, in degrees";
        param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        this->declare_parameter("compass_error_degrees", 5.0, param_desc);
      }

      // compass error factor
      {
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Factor to multiply compass error by when calculating covariance matrix, higher values make the compass less sensitive to errors";
        param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        this->declare_parameter("compass_error_factor", 1.0, param_desc);
      }

      {
        // camera_yaw_degrees
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Camera yaw correction in degrees";
        param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        
        rcl_interfaces::msg::FloatingPointRange range;
        range.set__from_value(-90.0).set__to_value(90.0);

        param_desc.floating_point_range= {range};

        this->declare_parameter("camera_yaw_degrees", -2.5, param_desc);

      }

      {
        // camera_pitch_degrees
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Camera pitch correction in degrees";
        param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        
        rcl_interfaces::msg::FloatingPointRange range;
        range.set__from_value(-90.0).set__to_value(90.0);

        param_desc.floating_point_range= {range};

        this->declare_parameter("camera_pitch_degrees", -2.5, param_desc);

      }

      {
        // camera_roll_degrees
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Camera roll correction in degrees";
        param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        
        rcl_interfaces::msg::FloatingPointRange range;
        range.set__from_value(-90.0).set__to_value(90.0);

        param_desc.floating_point_range= {range};

        this->declare_parameter("camera_roll_degrees", -2.5, param_desc);
      }


      {
        // camera_x
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Tf x distance from base_link to camera in meters";
        param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        
        this->declare_parameter("camera_x", 0.26, param_desc);
      }

      {
        // camera_y
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Tf y distance from base_link to camera in meters";
        param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;

        this->declare_parameter("camera_y", 0.0, param_desc);
      }

      {
        // camera_z
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Tf z distance from base_link to camera in meters";
        param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;

        this->declare_parameter("camera_z", 0.120, param_desc);
      }
      {
        // lidar_x
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Tf x distance from base_link to lidar in meters";
        param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        
        this->declare_parameter("lidar_x", 0.17, param_desc);
      }

      {
        // lidar_y
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Tf y distance from base_link to lidar in meters";
        param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;

        this->declare_parameter("lidar_y", 0.0, param_desc);
      }

      {
        // lidar_z
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Tf z distance from base_link to lidar in meters";
        param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;

        this->declare_parameter("lidar_z", 0.175, param_desc);
      }

      {
        // front_left_meters_per_tick
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Meters per tick for front left wheel";
        param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        
        this->declare_parameter("front_left_meters_per_tick", 0.002432, param_desc);
      }

      {
        // front_right_meters_per_tick
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Meters per tick for front right wheel";
        param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        
        this->declare_parameter("front_right_meters_per_tick", 0.002432, param_desc);
      }


      {
        // motor_meters_per_tick
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Meters per tick for motor sensor";
        param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        
        this->declare_parameter("motor_meters_per_tick", 0.002650, param_desc);
      }


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

      parameters_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&Car::parameters_callback, this, std::placeholders::_1));

      // read parameters
      this->get_parameter("front_left_meters_per_tick", front_left_wheel_.meters_per_tick);
      this->get_parameter("front_right_meters_per_tick", front_right_wheel_.meters_per_tick);
      this->get_parameter("motor_meters_per_tick", motor_.meters_per_tick);

      vector<double> velocity_to_esc_vector;
      this->get_parameter("velocity_to_esc_lookup", velocity_to_esc_vector);
      velocity_to_esc_lookup_table = std::make_unique<LookupTable>(velocity_to_esc_vector);

      vector<double> curvature_to_str_vector  ;
      this->get_parameter("curvature_to_str_lookup", curvature_to_str_vector);
      curvature_to_str_lookup_table = std::make_unique<LookupTable>(curvature_to_str_vector);


      fl_speedometer_publisher_ = this->create_publisher<car_msgs::msg::Speedometer> ("car/speedometers/fl", 10);
      fr_speedometer_publisher_ = this->create_publisher<car_msgs::msg::Speedometer> ("car/speedometers/fr", 10);
      motor_speedometer_publisher_ = this->create_publisher<car_msgs::msg::Speedometer> ("car/speedometers/motor", 10);
      ackerman_fr_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped> ("car/ackermann/fr", 10);
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      rc_command_publisher_ = this->create_publisher<car_msgs::msg::RcCommand>("car/rc_command", 1);

      car_update_subscription_ = this->create_subscription<car_msgs::msg::Update>(
      "car/update", rclcpp::SensorDataQoS(), std::bind(&Car::car_update_topic_callback, this, _1));

      cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 1, std::bind(&Car::cmd_vel_topic_callback, this, _1));

      odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("car/odom", 10);
      imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("car/imu", 10);
      compass_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("car/compass", 10);

      reset_service_ = this->create_service<std_srvs::srv::Empty>("car/reset",std::bind(&Car::reset_service_callback, this, _1, _2) );
      

      velocity_pid.k_p = 1.0;
      velocity_pid.k_i = 0.0;
      velocity_pid.k_d = 10.0;

    }

  private:
    float esc_us_float = 1500;
    float str_us_float = 1500;
    std::unique_ptr<LookupTable> curvature_to_str_lookup_table;
    std::unique_ptr<LookupTable> velocity_to_esc_lookup_table;



    int steering_for_curvature(Angle theta_per_meter) const;
    int esc_for_velocity(double v) const;

    Angle angle_for_steering(int str);
    void car_update_topic_callback(const car_msgs::msg::Update::SharedPtr d);

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> & parameters);


    int32_t update_count_ = 0;

    // car constants for blue-crash
    // todo: get from parameter server
    const float front_wheelbase_width_in_meters = 0.2413;
    const float rear_wheelbase_width_in_meters = 0.2667;
    const float wheelbase_length_in_meters = 0.33655;

    rclcpp::Publisher<car_msgs::msg::Speedometer>::SharedPtr fl_speedometer_publisher_;
    rclcpp::Publisher<car_msgs::msg::Speedometer>::SharedPtr fr_speedometer_publisher_;
    rclcpp::Publisher<car_msgs::msg::Speedometer>::SharedPtr motor_speedometer_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ackerman_fr_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr compass_publisher_;

    rclcpp::Subscription<car_msgs::msg::Update>::SharedPtr update_sub_;

    car_msgs::msg::Update::SharedPtr last_update_message_;

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

      auto pid = fork();
      if(pid==0) {
        const char * volume = "30"; // 0-200
        execlp("espeak", "espeak", "\"The odometer has been reset.\"","-a",volume, NULL);
        std::cout << "called espeak" << std::endl;
      }
      signal(SIGCHLD,SIG_IGN); // prevents child from becoming a zombie


      ackermann_.reset();
    }

    

    car_msgs::msg::Speedometer::SharedPtr speedometer_message;

    rclcpp::Subscription<car_msgs::msg::Update>::SharedPtr car_update_subscription_;
    rclcpp::Subscription<car_msgs::msg::Speedometer>::SharedPtr motor_speedometer_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;

    rclcpp::Publisher<car_msgs::msg::RcCommand>::SharedPtr rc_command_publisher_;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
};


int Car::steering_for_curvature(Angle theta_per_meter) const {
  if(curvature_to_str_lookup_table) {
    return (int)curvature_to_str_lookup_table->lookup(theta_per_meter.degrees());
  } else {
    RCLCPP_ERROR_ONCE(rclcpp::get_logger("car"), "steering_for_curvature() called with no curvature_to_str_lookup_table set. Using safe value of 1500.");
    return 1500;
  }
}

int Car::esc_for_velocity(double v)  const {
  if(velocity_to_esc_lookup_table) {
    return velocity_to_esc_lookup_table->lookup(v);
  } else {
    RCLCPP_ERROR_ONCE(rclcpp::get_logger("car"), "esc_for_velocity() called with no velocity_to_esc_lookup_table set. Using safe value of 1500.");
    return 1500;
  }
}


Angle Car::angle_for_steering(int str) {
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

rcl_interfaces::msg::SetParametersResult Car::parameters_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & parameter : parameters) {
    if (parameter.get_name() == "velocity_k_p") {
      velocity_pid.k_p = parameter.as_double();
      continue;
    }
    if (parameter.get_name() == "velocity_to_esc_lookup") {
      velocity_to_esc_lookup_table = std::make_unique<LookupTable>(parameter.as_double_array());
      continue;
    }

    if (parameter.get_name() == "curvature_to_str_lookup") {
      curvature_to_str_lookup_table = std::make_unique<LookupTable>(parameter.as_double_array());
      continue;
    }
  }
  return result;
}

void Car::car_update_topic_callback(const car_msgs::msg::Update::SharedPtr d){
    RCLCPP_INFO_ONCE(this->get_logger(), "Running and receiving car update messages");

    ++update_count_;

    Angle yaw = Angle::degrees(d->mpu_deg_yaw);
    Angle pitch = Angle::degrees(d->mpu_deg_pitch);
    Angle roll = Angle::degrees(d->mpu_deg_roll);

    front_left_wheel_.update_from_sensor(d->us, d->odo_fl_a, d->odo_fl_a_us,
                                        d->odo_fl_b, d->odo_fl_b_us);
    motor_.update_from_sensor(d->us, d->motor_odo, d->motor_us);
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
        //if(fabs(wheel_distance_meters)>0) {
            // todo: use the ackermann model to calculate the outside wheel angle
            Angle outside_wheel_angle = angle_for_steering(d->rx_str);
            ackermann_.move_right_wheel(outside_wheel_angle, wheel_distance_meters,
                                    yaw);
        //} 
    }

    // you can detect whether the Traxxas power switch is on by looking at rx signals
    bool powertrain_on = (d->rx_esc > 0 && d->rx_str > 0);
    
    geometry_msgs::msg::PoseStamped pose_msg;

    auto stamp = now(); // was d->header.stamp;

    Point rear_position = ackermann_.rear_position();
    
    pose_msg.header.stamp = stamp;
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

    // publish to odometry
    {
      static nav_msgs::msg::Odometry last_odom;

      nav_msgs::msg::Odometry odom;
      odom.header.stamp = stamp;
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_footprint";
      odom.pose.pose.position.x = rear_position.x;
      odom.pose.pose.position.y = rear_position.y;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation.x = q.x();
      odom.pose.pose.orientation.y = q.y();
      odom.pose.pose.orientation.z = q.z();
      odom.pose.pose.orientation.w = q.w();
      odom.pose.covariance = 
        {
          .1, 0, 0, 0, 0, 0,
          0, .1, 0, 0, 0, 0,
          0, 0, .1, 0, 0, 0,
          0, 0, 0, .1, 0, 0,
          0, 0, 0, 0, .1, 0,
          0, 0, 0, 0, 0, .1
        };


      // set the twist and publish only if we have a recent last_transform
      double dt = (stamp - last_odom.header.stamp).seconds();
      if(dt > 0.0 && dt < 0.1) {
        
        // 0.033 deg/s rms noise according to mpu6050 spec
        // https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
        double std_angular = 0.033 * M_PI / 180.0;
        double cov_angular = std_angular * std_angular;

        // twist is relative to the car
        double v_x = ackermann_.dx / dt;
        double v_y = ackermann_.dy / dt;
        odom.twist.twist.linear.x = v_x;
        odom.twist.twist.linear.y = v_y;

        // Convert quaternions to roll, pitch, yaw angles
        tf2::Quaternion q_odom, q_odom_last;
        tf2::fromMsg(odom.pose.pose.orientation, q_odom);
        tf2::fromMsg(last_odom.pose.pose.orientation, q_odom_last);

        double roll, pitch, yaw, roll_last, pitch_last, yaw_last;
        tf2::Matrix3x3(q_odom).getRPY(roll, pitch, yaw);
        tf2::Matrix3x3(q_odom_last).getRPY(roll_last, pitch_last, yaw_last);

        // Calculate the angular velocity        
        odom.twist.twist.angular.x = (roll - roll_last) / dt;
        odom.twist.twist.angular.y = (pitch - pitch_last) / dt;
        odom.twist.twist.angular.z = (yaw - yaw_last) / dt;

        // calculate covariances
        double std_vx = v_x * 0.01;
        double std_vy = v_y * 0.01;
        double std_vz = 0.0;

        

        odom.twist.covariance = 
          //         {
          //   0.1, 0, 0, 0, 0, 0,
          //   0, 0.1, 0, 0, 0, 0,
          //   0, 0, 0.1, 0, 0, 0,
          //   0, 0, 0, 0.1, 0, 0,
          //   0, 0, 0, 0, 0.1, 0,
          //   0, 0, 0, 0, 0, 0.1
          // };

          {
            std_vx * std_vx, 0, 0, 0, 0, 0,
            0, std_vy * std_vy, 0, 0, 0, 0,
            0, 0, std_vz  * std_vz, 0, 0, 0,
            0, 0, 0, cov_angular, 0, 0,
            0, 0, 0, 0, cov_angular, 0,
            0, 0, 0, 0, 0, cov_angular
          };

        odom_publisher_->publish(odom);

        // publish imu
        {
          sensor_msgs::msg::Imu imu;
          // get orientation from the mag_deg_yaw
          tf2::Quaternion q;
          q.setRPY(0, 0, d->mpu_deg_yaw * M_PI / 180.0);

          
          imu.header.stamp = stamp;
          imu.header.frame_id = "base_footprint";
          imu.orientation.x = q.x();
          imu.orientation.y = q.y();
          imu.orientation.z = q.z();
          imu.orientation.w = q.w();
          imu.orientation_covariance = 
            {
              0.1, 0, 0,
              0, 0.1, 0,
              0, 0, 0.1
            };

          imu.angular_velocity.x = odom.twist.twist.angular.x;
          imu.angular_velocity.y = odom.twist.twist.angular.y;
          imu.angular_velocity.z = odom.twist.twist.angular.z;
          imu.angular_velocity_covariance = 
            {
              cov_angular, 0, 0,
              0, cov_angular, 0,
              0, 0, cov_angular
            };

          imu.linear_acceleration.x = d->ax;
          imu.linear_acceleration.y = d->ay;
          imu.linear_acceleration.z = 0.0;
          imu.linear_acceleration_covariance = 
            {
              0.1, 0, 0,
              0, 0.1, 0,
              0, 0, 0.1
            };

          imu_publisher_->publish(imu);
        }

        // publish imu for magnetometer (will only have orientation)
        {
          sensor_msgs::msg::Imu compass;
          tf2::Quaternion q;
          double compass_correction_degrees;
          get_parameter<double>("compass_correction_degrees", compass_correction_degrees);

          double mag_x_min, mag_x_max, mag_y_min, mag_y_max;

          get_parameter("mag_x_min", mag_x_min);
          get_parameter("mag_x_max", mag_x_max);
          get_parameter("mag_y_min", mag_y_min);
          get_parameter("mag_y_max", mag_y_max);


          // scale range of -650 to 900 to -1 to 1
          double mag_x = mapRange(mag_x_min, mag_x_max, -1, 1, d->mag_x);
          double mag_y = mapRange(mag_y_min, mag_y_max, -1, 1, d->mag_y);
          double compass_radians = atan2(mag_y, mag_x);
          q.setRPY(0, 0, compass_radians + compass_correction_degrees * M_PI / 180.0);

          compass.header.stamp = stamp;
          compass.header.frame_id = "base_footprint";
          compass.orientation.x = q.x();
          compass.orientation.y = q.y();
          compass.orientation.z = q.z();
          compass.orientation.w = q.w();
          double compass_error_degrees;
          get_parameter<double>("compass_error_degrees", compass_error_degrees);
          double compass_error_factor;
          get_parameter<double>("compass_error_factor", compass_error_factor);
          double compass_error_radians = compass_error_degrees * M_PI / 180.0;
          double compass_std = compass_error_radians * compass_error_factor;
          double compass_covariance = compass_std * compass_std;
          compass.orientation_covariance = 
            {
              compass_covariance, 0, 0,
              0, compass_covariance, 0,
              0, 0, compass_covariance
            };
          // mark all other readings as invalid
          compass.angular_velocity_covariance = 
            {
              -1, 0, 0,
              0, -1, 0,
              0, 0, -1
            };
          compass.linear_acceleration_covariance = 
            {
              -1, 0, 0,
              0, -1, 0,
              0, 0, -1
            };
            
          compass_publisher_->publish(compass);
        } 


      }
  
      last_odom = odom;

    }



    std::vector<geometry_msgs::msg::TransformStamped> tf_msgs;

    // odom->base_footprint
    {
      geometry_msgs::msg::TransformStamped tf_msg;

      tf_msg.header.frame_id = "odom";
      tf_msg.child_frame_id = "base_footprint";
      tf_msg.header.stamp = stamp;
      tf_msg.transform.translation.x = rear_position.x;
      tf_msg.transform.translation.y = rear_position.y;
      tf_msg.transform.translation.z = 0.0;
      tf_msg.transform.rotation.x = q.x();
      tf_msg.transform.rotation.y = q.y();
      tf_msg.transform.rotation.z = q.z();
      tf_msg.transform.rotation.w = q.w();

      // don't publish since we are using ekf
      // todo: make this configurable
      // tf_msgs.push_back(tf_msg);
    }

    // base_footprint->base_link
    {
      geometry_msgs::msg::TransformStamped tf_msg;

      tf2::Quaternion q;
      q.setRPY(roll.radians(), -pitch.radians(), 0);

      tf_msg.header.frame_id = "base_footprint";
      tf_msg.child_frame_id = "base_link";
      tf_msg.header.stamp = stamp;
      tf_msg.transform.translation.x = 0.0;
      tf_msg.transform.translation.y = 0.0;
      tf_msg.transform.translation.z = 0.06;
      tf_msg.transform.rotation.x = q.x();
      tf_msg.transform.rotation.y = q.y();
      tf_msg.transform.rotation.z = q.z();
      tf_msg.transform.rotation.w = q.w();
      tf_msgs.push_back(tf_msg);
    }

    // base_link->laser_scanner_link
    {

      double lidar_x = 0.0;
      get_parameter<double>("lidar_x", lidar_x);

      double lidar_y = 0.0;
      get_parameter<double>("lidar_y", lidar_y);

      double lidar_z = 0.0;
      get_parameter<double>("lidar_z", lidar_z);

      geometry_msgs::msg::TransformStamped tf_msg;

      

      tf_msg.header.frame_id = "base_link";
      tf_msg.child_frame_id = "laser_scanner_link";
      tf_msg.header.stamp = stamp;
      tf_msg.transform.translation.x = lidar_x;
      tf_msg.transform.translation.y = lidar_y;
      tf_msg.transform.translation.z = lidar_z;
      tf_msg.transform.rotation.x = 0.0;
      tf_msg.transform.rotation.y = 1.0;
      tf_msg.transform.rotation.z = 0.0;
      tf_msg.transform.rotation.w = 0.0;
      tf_msgs.push_back(tf_msg);
    }

    // base_link->oak_rgb_camera_frame
    {
      tf2::Quaternion q;
      double camera_yaw_degrees;
      get_parameter<double>("camera_yaw_degrees", camera_yaw_degrees);

      double camera_pitch_degrees;
      get_parameter<double>("camera_pitch_degrees", camera_pitch_degrees);

      double camera_roll_degrees;
      get_parameter<double>("camera_roll_degrees", camera_roll_degrees);

      double camera_x;
      get_parameter<double>("camera_x", camera_x);

      double camera_y;
      get_parameter<double>("camera_y", camera_y);

      double camera_z;
      get_parameter<double>("camera_z", camera_z);

      q.setRPY(camera_roll_degrees * M_PI/180, camera_pitch_degrees*M_PI/180, camera_yaw_degrees*M_PI/180.0);
      geometry_msgs::msg::TransformStamped tf_msg;

      tf_msg.header.frame_id = "base_link";
      tf_msg.child_frame_id = "oak_rgb_camera_frame";
      tf_msg.header.stamp = stamp;
      tf_msg.transform.translation.x = camera_x;
      tf_msg.transform.translation.y = camera_y;
      tf_msg.transform.translation.z = camera_z;
      tf_msg.transform.rotation.x = q.x();
      tf_msg.transform.rotation.y = q.y();
      tf_msg.transform.rotation.z = q.z();
      tf_msg.transform.rotation.w = q.w();
      tf_msgs.push_back(tf_msg);
    }

    // oak_rgb_camera_frame->oak_rgb_camera_optical_frame
    //
    // Camera optics use a weird orientation that is x goes to the right
    // and y goes down. This frame handles that.
    {
      tf2::Quaternion q;
      q.setRPY(-M_PI/2.0, 0, -M_PI/2.0);
      geometry_msgs::msg::TransformStamped tf_msg;

      tf_msg.header.frame_id = "oak_rgb_camera_frame";
      tf_msg.child_frame_id = "oak_rgb_camera_optical_frame";
      tf_msg.header.stamp = stamp;
      tf_msg.transform.translation.x = 0.0;
      tf_msg.transform.translation.y = 0.0;
      tf_msg.transform.translation.z = 0.0;
      tf_msg.transform.rotation.x = q.x();
      tf_msg.transform.rotation.y = q.y();
      tf_msg.transform.rotation.z = q.z();
      tf_msg.transform.rotation.w = q.w();
      tf_msgs.push_back(tf_msg);
    }

    tf_broadcaster_->sendTransform(tf_msgs);

    // send static transforms
    last_fr_ = fr;
    last_fl_ = fl;
    last_motor_ = motor;

    if(powertrain_on && cmd_vel_message && (now()-cmd_vel_receive_time) < 500ms ) {

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

    // remember the last message for the next iteration
    last_update_message_ = d;

}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Car>());
  rclcpp::shutdown();
  return 0;
}
#pragma once

#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/typesupport_helpers.hpp>
//#include <rosbag2_cpp/types/introspection_message.hpp>
#include <rosidl_typesupport_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include <vector>
#include  <fastcdr/Cdr.h>

void stream_json(
  std::ostream & stream, 
  eprosima::fastcdr::Cdr &cdr, 
  const rosidl_message_type_support_t* type_data);

class JsonEncoder{

public:
  JsonEncoder()
 {}

  void set_message_type(const std::string& type_name);

  void stream_json(std::ostream & stream, const rcl_serialized_message_t *msg) const;


private:
  const rosidl_message_type_support_t * type_support_;
};


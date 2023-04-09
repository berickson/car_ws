#include "json_encoder.hpp"

#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/types/introspection_message.hpp>
#include <rcutils/time.h>
#include <functional>
#include <cmath>
#include  <fastcdr/Cdr.h>


void JsonEncoder::set_message_type(
  const std::string &type_name)
{
  auto library = rclcpp::get_typesupport_library(type_name, rosidl_typesupport_introspection_cpp::typesupport_identifier);
  type_support_ =
    rosbag2_cpp::get_typesupport_handle(type_name, rosidl_typesupport_introspection_cpp::typesupport_identifier, library);
}


template <typename T> inline T deserialize(eprosima::fastcdr::Cdr& cdr)
{
  T tmp;
  cdr.deserialize(tmp);
  return tmp;
}

void stream_json(
  std::ostream & stream, 
  eprosima::fastcdr::Cdr &cdr, 
  const rosidl_message_type_support_t* type_data)
  {
    bool pretty = true;

    stream << "{";

    using namespace rosidl_typesupport_introspection_cpp;
    const auto* members = static_cast<const MessageMembers*>(type_data->data);

    for(size_t index = 0; index < members->member_count_; index++)
    {
      if(index > 0) {
        stream << ",";
        if(pretty) {
          stream << std::endl;
        }
      }
      const MessageMember& member = members->members_[index];
      stream << "\""<< member.name_ << "\": ";

      size_t array_size = 1;

      if(member.is_array_)
      {
        if( member.array_size_ == 0)
        {
          array_size = size_t(deserialize<uint32_t>(cdr));
        }
        else{
          array_size = member.array_size_;
        }
      }

      for (size_t a=0; a < array_size; a++)
      {
        if( member.is_array_ )
        {
          //
        }

        if(member.type_id_ != ROS_TYPE_MESSAGE && member.type_id_ != ROS_TYPE_STRING)
        {
          double value = 0;
          switch( member.type_id_)
          {
            case ROS_TYPE_FLOAT:   stream << deserialize<float>(cdr); break;
            case ROS_TYPE_DOUBLE:  stream << deserialize<double>(cdr); break;
            case ROS_TYPE_INT64:   stream << deserialize<int64_t>(cdr); break;
            case ROS_TYPE_INT32:   stream << deserialize<int32_t>(cdr); break;
            case ROS_TYPE_INT16:   stream << deserialize<int16_t>(cdr); break;
            case ROS_TYPE_INT8:    stream << deserialize<int8_t>(cdr); break;
            case ROS_TYPE_UINT64:  stream << deserialize<uint64_t>(cdr); break;
            case ROS_TYPE_UINT32:  stream << deserialize<uint32_t>(cdr); break;
            case ROS_TYPE_UINT16:  stream << deserialize<uint16_t>(cdr); break;
            case ROS_TYPE_UINT8:   stream << deserialize<uint8_t>(cdr); break;
            case ROS_TYPE_BOOLEAN: stream << deserialize<bool>(cdr); break;
          }
        }
        else if(member.type_id_ == ROS_TYPE_STRING)
        {
          std::string str;
          cdr.deserialize( str );
          stream << "\"" << str << "\"";
        }
        else if(member.type_id_ == ROS_TYPE_MESSAGE)
        {
          ::stream_json(stream, cdr, member.members_);
        }
      }
    }
    stream << "}";
  }


void JsonEncoder::stream_json(std::ostream & stream, const rcutils_uint8_array_t *msg) const
{
  // get cdr reader from buffer
  eprosima::fastcdr::FastBuffer buffer( reinterpret_cast<char*>(msg->buffer), msg->buffer_length);
  eprosima::fastcdr::Cdr cdr(buffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
                             eprosima::fastcdr::Cdr::DDS_CDR);
  cdr.read_encapsulation();


  ::stream_json(stream, cdr, type_support_);
}

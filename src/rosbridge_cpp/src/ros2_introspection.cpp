#include <ros2_introspection.hpp>

#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/types/introspection_message.hpp>
#include <rcutils/time.h>
#include <functional>
#include <cmath>
#include  <fastcdr/Cdr.h>


void Parser::registerMessageType(
  const std::string &message_identifier,
  const std::string &type_name)
{
  if( _registered_messages.count(message_identifier) != 0)
  {
    return;
  }


  auto library = rclcpp::get_typesupport_library(type_name, rosidl_typesupport_introspection_cpp::typesupport_identifier);
  auto type_support =
    rosbag2_cpp::get_typesupport_handle(type_name, rosidl_typesupport_introspection_cpp::typesupport_identifier, library);
  
  std::pair<std::string,const rosidl_message_type_support_t* > pair;
  pair.first = message_identifier;
  pair.second = type_support;
  _registered_messages.insert( std::move(pair) );
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
            case ROS_TYPE_FLOAT:   value = double(deserialize<float>(cdr)); break;
            case ROS_TYPE_DOUBLE:  value = double(deserialize<double>(cdr)); break;

            case ROS_TYPE_INT64:   value = double(deserialize<int64_t>(cdr)); break;
            case ROS_TYPE_INT32:   value = double(deserialize<int32_t>(cdr)); break;
            case ROS_TYPE_INT16:   value = double(deserialize<int16_t>(cdr)); break;
            case ROS_TYPE_INT8:    value = double(deserialize<int8_t>(cdr)); break;

            case ROS_TYPE_UINT64:  value = double(deserialize<uint64_t>(cdr)); break;
            case ROS_TYPE_UINT32:  value = double(deserialize<uint32_t>(cdr)); break;
            case ROS_TYPE_UINT16:  value = double(deserialize<uint16_t>(cdr)); break;
            case ROS_TYPE_UINT8:   value = double(deserialize<uint8_t>(cdr)); break;

            case ROS_TYPE_BOOLEAN: value = double(deserialize<bool>(cdr)); break;
          }
          stream << value;
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
      } // end for array
    } // end for members
    stream << "}";
  }


void Parser::stream_json(const std::string & msg_identifier, std::ostream & stream, const rcutils_uint8_array_t *msg) const
{
  const auto message_info_it = _registered_messages.find(msg_identifier);
  if(message_info_it == _registered_messages.end())
  {
    throw std::runtime_error("Message identifier not registered");
  }

  // get cdr reader from buffer
  eprosima::fastcdr::FastBuffer buffer( reinterpret_cast<char*>(msg->buffer), msg->buffer_length);
  eprosima::fastcdr::Cdr cdr(buffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
                             eprosima::fastcdr::Cdr::DDS_CDR);
  cdr.read_encapsulation();

  auto& type_support = message_info_it->second;


  ::stream_json(stream, cdr, type_support);
}


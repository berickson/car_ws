#include "json_encoder.hpp"

#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
//#include <rosbag2_cpp/typesupport_helpers.hpp>
//#include <rosbag2_cpp/types/introspection_message.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/time.h>
#include <functional>
#include <cmath>
#include  <fastcdr/Cdr.h>
#include "nlohmann/json.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"


void JsonEncoder::set_message_type(
  const std::string &type_name)
{
  auto library = rclcpp::get_typesupport_library(type_name, rosidl_typesupport_introspection_cpp::typesupport_identifier);
  type_support_ =
    rclcpp::get_typesupport_handle(type_name, rosidl_typesupport_introspection_cpp::typesupport_identifier, *library);
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

        stream << "[";
      }

      for (size_t a=0; a < array_size; a++)
      {
        if(a>0) {
          stream << ", ";
        }
        if( member.is_array_ )
        {
          //
        }

        switch( member.type_id_)
        {
          case ROS_TYPE_FLOAT:
            {
              auto f = deserialize<float>(cdr);
              if(std::isnan(f) || std::isinf(f)) {
                stream << "null";
              } else {
                stream << f;
              }
            }
            break;
          case ROS_TYPE_DOUBLE:
            {
              auto d = deserialize<double>(cdr);
              if(std::isnan(d) || std::isinf(d)) {
                stream << "null";
              } else {
                stream << d;
              }
            }
            break;
          case ROS_TYPE_INT64:   stream << deserialize<int64_t>(cdr); break;
          case ROS_TYPE_INT32:   stream << deserialize<int32_t>(cdr); break;
          case ROS_TYPE_INT16:   stream << deserialize<int16_t>(cdr); break;
          case ROS_TYPE_INT8:    stream << (int)deserialize<int8_t>(cdr); break;
          case ROS_TYPE_UINT64:  stream << deserialize<uint64_t>(cdr); break;
          case ROS_TYPE_UINT32:  stream << deserialize<uint32_t>(cdr); break;
          case ROS_TYPE_UINT16:  stream << deserialize<uint16_t>(cdr); break;
          case ROS_TYPE_UINT8:   stream << (int)deserialize<uint8_t>(cdr); break;
          case ROS_TYPE_BOOLEAN: stream << deserialize<bool>(cdr); break;
          case ROS_TYPE_STRING:  
            {
              stream << "\"" << deserialize<std::string>(cdr) << "\""; 
            }
            break;
          case ROS_TYPE_MESSAGE: ::stream_json(stream, cdr, member.members_); break;
          default:
            std::cout << "Unknown ROS Type" << member.type_id_ << std::endl;
        }
      }
    
      if(member.is_array_) {
        stream << "]";
      }
    }
    stream << "}";
  }


void JsonEncoder::stream_json(std::ostream & stream, const rcl_serialized_message_t *msg) const
{
  // get cdr reader from buffer
  eprosima::fastcdr::FastBuffer buffer( reinterpret_cast<char*>(msg->buffer), msg->buffer_length);
  eprosima::fastcdr::Cdr cdr(buffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
                             eprosima::fastcdr::Cdr::DDS_CDR);
  cdr.read_encapsulation();


  ::stream_json(stream, cdr, type_support_);
}


using namespace rosidl_typesupport_introspection_cpp;
void serialize_json_to_cdr(
  eprosima::fastcdr::Cdr &cdr, 
  nlohmann::json & json, 
  const MessageMembers * members) 
{
  //std::cout << "serializing json to cdr" << std::endl;
  for(uint32_t i=0; i<members->member_count_ ; ++i) {
    auto member = members->members_[i];
    //std::cout << "streaming " << member.name_ << std::endl;
    auto e = json[member.name_];
    switch(member.type_id_) {
      case ROS_TYPE_INT64:   cdr.serialize( e.get<int64_t>()); break;  
      case ROS_TYPE_INT32:   cdr.serialize( e.get<int32_t>()); break;
      case ROS_TYPE_INT16:   cdr.serialize( e.get<int16_t>()); break;   
      case ROS_TYPE_INT8:    cdr.serialize( e.get<int8_t>()); break;

      case ROS_TYPE_UINT64:  cdr.serialize( e.get<uint64_t>()); break;
      case ROS_TYPE_UINT32:  cdr.serialize( e.get<uint32_t>()); break;
      case ROS_TYPE_UINT16:  cdr.serialize( e.get<uint16_t>()); break;
      case ROS_TYPE_UINT8:   cdr.serialize( e.get<uint8_t>()); break;

      case ROS_TYPE_FLOAT:   cdr.serialize( e.get<float>()); break; // same as ROS_TYPE_FLOAT32
      case ROS_TYPE_DOUBLE:  cdr.serialize( e.get<double>()); break; // same as ROS_TYPE_FLOAT64

      case ROS_TYPE_STRING:  cdr.serialize( e.get<std::string>()); break; // todo: decode
      case ROS_TYPE_MESSAGE: serialize_json_to_cdr(cdr, e, (MessageMembers *) member.members_->data); break; 
      default: break;
    }
  }

}
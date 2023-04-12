#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "json_encoder.hpp"
//#include <rosidl_runtime_c/service_type_support_struct.h>

#include "Simple-WebSocket-Server/server_ws.hpp"
#include <future>

#include "nlohmann/json.hpp"

#include  <fastcdr/Cdr.h>

using WsServer = SimpleWeb::SocketServer<SimpleWeb::WS>;
using namespace std::chrono_literals;
using std::placeholders::_1;


class RosbridgeCppNode : public rclcpp::Node
{
  public:
    RosbridgeCppNode()
    : Node("rosbridge_cpp"), count_(0)
    {
      start_webservice();
      RCLCPP_INFO(this->get_logger(), "constructor complete");

    }

  private:

    WsServer server;
    std::thread server_thread;


    struct SubscriptionInfo {
      rclcpp::Node * node;
      std::string topic;
      std::string id;
      std::string type;
      std::shared_ptr<rclcpp::GenericSubscription> subscription;
      std::shared_ptr<WsServer::Connection> connection;
      int32_t throttle_rate_ms = 0;
      std::chrono::time_point<std::chrono::system_clock, std::chrono::duration<double>> last_sent_time{};
      void generic_callback(std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) {
        if(std::chrono::system_clock::now()-last_sent_time < std::chrono::milliseconds(throttle_rate_ms)) {
          return;
        }
        last_sent_time = std::chrono::system_clock::now();

        JsonEncoder json_encoder;
        json_encoder.set_message_type(type);

        std::stringstream ss_json;
        ss_json << "{ \"op\": \"publish\", \"id\": \"" << id << "\", \"topic\":\""<< topic << "\", \"msg\":";
        json_encoder.stream_json(ss_json, &serialized_msg->get_rcl_serialized_message());
        ss_json << "}";

        connection->send(ss_json.str());
      }
    };

    struct PublisherInfo {
      rclcpp::Node * node;
      std::string topic;
      std::string id;
      std::string type;
      std::shared_ptr<rclcpp::GenericPublisher> publisher;
      std::shared_ptr<WsServer::Connection> connection;
      std::shared_ptr<rcpputils::SharedLibrary> type_support_library;
    };

    std::map<std::string, std::shared_ptr<SubscriptionInfo>> subscriptions_;
    std::map<std::string, std::shared_ptr<PublisherInfo>> publishers_;

    void advertise(nlohmann::json & json, std::shared_ptr<WsServer::Connection> connection ) {
      std::string topic = json["topic"];
      std::string type = json["type"];
      std::string id = json["id"];
      
      if(publishers_.find(topic) != publishers_.end()) {
        std::cout << " topic already published " << std::endl;
      }

      auto publisher_info = std::make_shared<PublisherInfo>();
      publisher_info->topic = topic;
      publisher_info->type = type;
      publisher_info->id = id;
      publisher_info->connection = connection;
      publisher_info->type_support_library = rclcpp::get_typesupport_library(type, rosidl_typesupport_introspection_cpp::typesupport_identifier);

      if(publisher_info->type_support_library) {
        std::cout << "got type_support_library for " << type << std::endl;
      } else {
        std::cout << "*** failed to get type_support_library for " << type << std::endl;
      }

      std::cout << "advertising topic " << topic
                << " type " << type 
                << " id " << id 
                << std::endl;
      
      publisher_info->publisher = this->create_generic_publisher(
        topic, 
        type, 
        rclcpp::SystemDefaultsQoS());
      std::cout << "before added publisher for " << topic << std::endl;
      publishers_[topic] = publisher_info;
      std::cout << "added publisher for " << topic << std::endl;
    }

    void publish(nlohmann::json & json) {
      std::string topic = json["topic"];
      auto publisher_info = publishers_[topic];
      if(!publisher_info) {
        std::cout << "couldn't find publisher for " << topic << std::endl;
        return;
      }
      std::string message_type = publisher_info->type;

      auto type_support =
          rclcpp::get_typesupport_handle(message_type, rosidl_typesupport_introspection_cpp::typesupport_identifier, *(publisher_info->type_support_library));


      
      std::cout << "message type " << message_type << " type_support->typesupport_identifier " << type_support->typesupport_identifier << std::endl;

      using namespace rosidl_typesupport_introspection_cpp;
      const auto* members = static_cast<const MessageMembers*>(type_support->data);
      
      std::cout << "1 publish creating SerializedMessage with size " << members->size_of_ << std::endl;
      rclcpp::SerializedMessage message(members->size_of_);
      std::cout << "2 publish creating SerializedMessage" << std::endl;
      auto msg=message.get_rcl_serialized_message();
      std::cout << "3 publish creating SerializedMessage" << std::endl;
      eprosima::fastcdr::FastBuffer buffer( reinterpret_cast<char*>(msg.buffer), msg.buffer_length);
      std::cout << "4 publish creating SerializedMessage" << std::endl;
      eprosima::fastcdr::Cdr cdr(
        buffer, 
        eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
        eprosima::fastcdr::Cdr::DDS_CDR);
      std::cout << "5 publish creating SerializedMessage" << std::endl;
      cdr.read_encapsulation();




      for(uint32_t i=0; i<members->member_count_ ; ++i) {
        auto & member =  members->members_[i];
      }

      std::cout << "done with publish" << std::endl;
      // type_support_handle->
      // publisher_info->publisher->publish(
    }



    void subscribe(nlohmann::json & json, std::shared_ptr<WsServer::Connection> connection) {
      std::string topic = json["topic"];
      std::string type;
      std::string id = json["id"];
      int throttle_rate_ms = 0;

      // get the message type
      if(json.contains("type")) {
         type = json["type"];
      } else {
        auto topics = get_topic_names_and_types();
        auto it_types = topics.find(topic);
        if(it_types != topics.end()) {
          type = it_types->second[0];
        }
        std::cout << "Couldn't find type for " << topic << std::endl;
      }

      try {
        throttle_rate_ms = json["throttle_rate"];
      } catch (...) 
      {}


      auto subscription_info = std::make_shared<SubscriptionInfo>();
      subscription_info->topic = topic;
      subscription_info->type = type;
      subscription_info->id = id;
      subscription_info->connection = connection;
      subscription_info->throttle_rate_ms = throttle_rate_ms;
      
      std::cout << "subscribing to topic " << topic 
                << " type " << type 
                << " throttle_rate_ms " << throttle_rate_ms 
                << " id " << id 
                << std::endl;

      subscription_info->subscription = this->create_generic_subscription(
        topic, 
        type, 
        rclcpp::SensorDataQoS(), 
        std::bind(
          &RosbridgeCppNode::SubscriptionInfo::generic_callback, 
          subscription_info.get(), 
          _1));

      subscriptions_[topic]=subscription_info;
    }

    void start_webservice() {
     
      server.config.port = 9090; // this seems to be the default port

      // configure endpoints
      auto &ws_endpoint = server.endpoint["^.*$"];
      ws_endpoint.on_message = [this](std::shared_ptr<WsServer::Connection> connection, std::shared_ptr<WsServer::InMessage> in_message) {
        try {
          auto json = nlohmann::json::parse(in_message->string());

          std::string op = json["op"];
          if(op=="subscribe") {
            subscribe(json, connection);
          } else if (op=="call_service"){
            std::cout << in_message->string() << std::endl;
            std::string service = json["service"];
            std::string type = json["type"];
            std::string command = "timeout 5 ros2 service call " + service + " " + type + " {}";
            std::cout << "executing: " << command << std::endl; 
            std::ignore = system(command.c_str());
          } else if (op=="advertise") {
            advertise(json, connection);
            std::cout << "advertise done" << std::endl;
          } else if (op=="publish") {
            std::cout << "calling publish " << std::endl;
            publish(json);
          }
          else {
            std::cout << "got op of \"" << op << "\"" << std::endl;
          }
        } catch (...) {
          std::cout << "Exception caught in websocket handler " << in_message->string() << std::endl;
        }
      };

      ws_endpoint.on_close = [this](std::shared_ptr<WsServer::Connection> connection, int status, const std::string & /*reason*/) {
        std::cout << "Server: Closed connection " << connection.get() << " with status code " << status << std::endl;
        for (auto it = subscriptions_.cbegin(); it != subscriptions_.cend() /* not hoisted */; /* no increment */) {
          if(it->second->connection == connection) {
            std::cout << "removing subscription to " << it->second->topic << std::endl;
            subscriptions_.erase(it++);
          } else {
            ++it;
          }
        }
      };

      // Start server and receive assigned port when server is listening for requests
      std::promise<unsigned short> server_port;
      server_thread = std::thread([this,  &server_port]() {
        // Start server
        server.start([&server_port](unsigned short port) {
          server_port.set_value(port);
        });
      });
      std::cout << "Server listening on port " << server_port.get_future().get() << std::endl
          << std::endl;
    }

    std::string topic_name_;
    std::string type_name_;
    JsonEncoder json_encoder_;    
    size_t count_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  {
    std::string message_type = "std_srvs/srv/Empty_Request";
    auto library = rclcpp::get_typesupport_library(message_type, "rosidl_typesupport_cpp");
    auto type_support_handle = rclcpp::get_typesupport_handle(message_type, "rosidl_typesupport_cpp", *library);

    if(type_support_handle) {
      std::cout << "**** got type support ****" << std::endl;
    } else {
      std::cout << "**** FAILED to get type support ****" << std::endl;
    }
  }

  rclcpp::spin(std::make_shared<RosbridgeCppNode>());
  rclcpp::shutdown();
  return 0;
}
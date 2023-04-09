#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "json_encoder.hpp"


#include "Simple-WebSocket-Server/server_ws.hpp"
#include <future>

#include "nlohmann/json.hpp"

#include  <fastcdr/Cdr.h>

using WsServer = SimpleWeb::SocketServer<SimpleWeb::WS>;
using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class RosbridgeCppNode : public rclcpp::Node
{
  public:
    RosbridgeCppNode()
    : Node("rosbridge_cpp"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&RosbridgeCppNode::timer_callback, this));
      auto topics = this->get_topic_names_and_types();
      topic_name_ = "/some_string";

      auto it_types = topics.find(topic_name_);
      if(it_types != topics.end()) {
        type_name_ = it_types->second[0];
        generic_subscription_ = this->create_generic_subscription(topic_name_, type_name_, rclcpp::SensorDataQoS(), std::bind(&RosbridgeCppNode::generic_callback, this, _1));
        RCLCPP_INFO(this->get_logger(), "Node done creating subsciption to %s %s", topic_name_.c_str(), type_name_.c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "Could not find topic");
      }

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
      void generic_callback(std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) {

        JsonEncoder json_encoder;
        json_encoder.set_message_type(type);

        std::stringstream ss_json;
        json_encoder.stream_json(ss_json, &serialized_msg->get_rcl_serialized_message());
        std::cout << ss_json.str() << std::endl;

      }
    };

    std::map<std::string, std::shared_ptr<SubscriptionInfo>> subscriptions_;


    void subscribe(nlohmann::json & json) {
      std::string topic = json["topic"];
      std::string type = json["type"];
      std::string id = json["id"];


      auto subscription_info = std::make_shared<SubscriptionInfo>();
      subscription_info->topic = topic;
      subscription_info->type = type;
      subscription_info->id = id;

      subscription_info->subscription = this->create_generic_subscription(topic, type, rclcpp::SensorDataQoS(), std::bind(&RosbridgeCppNode::SubscriptionInfo::generic_callback, subscription_info.get(), _1));

      subscriptions_[id]=subscription_info;
    }

    void start_webservice() {
     
      server.config.port = 9090; // this seems to be the default port

      // configure endpoints
      auto &ws_endpoint = server.endpoint["^.*$"];
      ws_endpoint.on_message = [this](std::shared_ptr<WsServer::Connection> connection, std::shared_ptr<WsServer::InMessage> in_message) {
        try {
          std::cout << "got message: " << in_message->string().c_str() << std::endl;
          auto json = nlohmann::json::parse(in_message->string());

          std::string op = json["op"];
          std::cout << "op: " << op  << std::endl;
          if(op=="subscribe") {
            subscribe(json);
          }
        } catch (...) {
          std::cout << "Exception caught in websocket handler" << std::endl;
        }
        // auto out_message = make_shared<string>(in_message->string());

        // connection->send(*out_message, [connection, out_message](const SimpleWeb::error_code &ec) {
        //   if(!ec)
        //     connection->send(*out_message); // Sent after the first send operation is finished
        // });
        // connection->send(*out_message); // Most likely queued. Sent after the first send operation is finished.
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

    void generic_callback(std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) {
      std::string message_type = "std_msgs/msg/String";

      auto handle = generic_subscription_->get_message_type_support_handle();
      RCLCPP_INFO(this->get_logger(), "typesupport identifier %s", handle.typesupport_identifier);
      static bool first_time = true;
      if(first_time) {
        
        json_encoder_.set_message_type(message_type);
        first_time = false;
      }

      std::stringstream ss_json;
      json_encoder_.stream_json(ss_json, &serialized_msg->get_rcl_serialized_message());
      std::cout << "JSON: " << ss_json.str() << std::endl;
    }

    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      publisher_->publish(message);
    }
    std::string topic_name_;
    std::string type_name_;
    rclcpp::TimerBase::SharedPtr timer_;
    JsonEncoder json_encoder_;    
    std::shared_ptr<rclcpp::GenericSubscription> generic_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // make socket server
  

  rclcpp::spin(std::make_shared<RosbridgeCppNode>());
  rclcpp::shutdown();
  return 0;
}
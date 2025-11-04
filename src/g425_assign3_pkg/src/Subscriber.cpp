#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "g425_assign3_interfaces_pkg/msg/imudata.hpp"

using namespace std::placeholders;
using String = std_msgs::msg::String;
class ESP32Subscriber : public rclcpp::Node
{
public:
  ESP32Subscriber() : Node("esp32_subscriber_node")
  {
    subscription_ = this->create_subscription<String>(
      "esp32_topic",
      10,
      std::bind(&ESP32Subscriber::topic_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "ESP32 subscriber initialized. Listening on /esp32_topic...");
  }

private:
  void topic_callback(const String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received from ESP32: '%s'", msg->data.c_str());
  }

  rclcpp::Subscription<String>::SharedPtr subscription_;
private:
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ESP32Subscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

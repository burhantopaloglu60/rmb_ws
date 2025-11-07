/*
Node description:
Lifecycle Node for handling IMU data from an ESP32.

This node subscribes to IMU messages published by an ESP32 on the `/esp32_topic`
and republishes them to `/imu_data` — intended for a database subscriber node.
 
The node have the following states:
- In the configuring state, it sets up publishers and subscribers.
- In the active state, it processes and republishes IMU data.
- In the inactive state, it pauses data publishing.

The lifecycle state is managed by a lifecycle manager.
*/ 

/*
--Software changes:
one line per change 
(1) created 30.10.2025: developer-Melissa van Leeuwen 
(2) edited 06.11.2025: developer-Rik van Velzen, functionality to receive wireless IMU data from ESP32 added
*/

#include <memory>
#include <string>
#include <arpa/inet.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "lifecycle_msgs/msg/transition.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <lifecycle_msgs/msg/state.hpp>

using namespace std::placeholders;
using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using CallbackReturn = LifecycleNodeInterface::CallbackReturn;

class IMULifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  IMULifecycleNode(const rclcpp::NodeOptions & options)
  : rclcpp_lifecycle::LifecycleNode("imu_lifecycle_node", options)
  {
    this->declare_parameter<bool>("connection_type", 1); // 0 for wired, 1 for wireless
    connection_type_ = this->get_parameter("connection_type").as_bool();
    
    RCLCPP_INFO(get_logger(), "Lifecycle node started in state: %s", this->get_current_state().label().c_str());
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Configuring IMU lifecycle node...");
    if(connection_type_ == 0){
      RCLCPP_INFO(this->get_logger(), "Wired connection selected.");
      // Subscriber on topic of ESP32
      subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu_data_esp",
      10,
      std::bind(&IMULifecycleNode::esp32Callback, this, _1)
      );

    } else 
    {
      RCLCPP_INFO(this->get_logger(), "Wireless connection selected.");
      this->declare_parameter<int>("port", 5005);
      port_ = this->get_parameter("port").as_int();

      this->declare_parameter<double>("tolerance", 1e-6);
      tolerance_ = this->get_parameter("tolerance").as_double();
      this->declare_parameter<int>("timer_period_ms", 200);
              timer_period_ms_ = this->get_parameter("timer_period_ms").as_int();

      connect_socket();

      // Timer for polling data
      timer_ = this->create_wall_timer(
          std::chrono::milliseconds(timer_period_ms_),
          std::bind(&IMULifecycleNode::receive_data, this)
      );
    }
    // Publisher to /imu_data so the subscriber can put the data into the database
    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & state)
  {
    LifecycleNode::on_activate(state);
    publisher_->on_activate();
    RCLCPP_INFO(get_logger(), "Node activated, ready to receive and publish IMU data.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state)
  {
    LifecycleNode::on_deactivate(state);
    publisher_->on_deactivate();
    RCLCPP_INFO(get_logger(), "Node deactivated.");
    return CallbackReturn::SUCCESS;
  }

private:
  void connect_socket()
  {
      sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
      if (sockfd_ < 0) {
          RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
          return;
      }

      sockaddr_in server_addr{};
      server_addr.sin_family = AF_INET;
      server_addr.sin_addr.s_addr = INADDR_ANY;
      server_addr.sin_port = htons(port_);

      if (bind(sockfd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
          RCLCPP_ERROR(this->get_logger(), "Failed to bind socket");
          close(sockfd_);
          return;
      }
      RCLCPP_INFO(this->get_logger(), "Listening for UDP packets on port %d...", port_);
  }
  void receive_data()
  {
      char buffer[256];
      sockaddr_in sender_addr{};
      socklen_t sender_len = sizeof(sender_addr);

      ssize_t bytes_received = recvfrom(sockfd_, buffer, sizeof(buffer) - 1, MSG_DONTWAIT,
                                        (struct sockaddr*)&sender_addr, &sender_len);
      float gx = 0, gy = 0, gz = 0, ax = 0, ay = 0, az = 0;
      if (bytes_received > 0) {
          buffer[bytes_received] = '\0'; // Null terminate
          if (sscanf(buffer, "%f,%f,%f,%f,%f,%f", &gx, &gy, &gz, &ax, &ay, &az) == 6) {
              
              std::vector<float> imu_data = {gx, gy, gz, ax, ay, az};
              if (is_all_zero(imu_data)) {
                  RCLCPP_WARN(this->get_logger(),
                      "Skipping zero IMU data packet.");
                  return;  // Do not publish
              }
              
              // publish_data(imu_data);
              sensor_msgs::msg::Imu imu_msg;
              imu_msg.header.stamp = this->now();
              imu_msg.header.frame_id = "imu_link";
              imu_msg.angular_velocity.x = gx;
              imu_msg.angular_velocity.y = gy;
              imu_msg.angular_velocity.z = gz;
              imu_msg.linear_acceleration.x = ax;
              imu_msg.linear_acceleration.y = ay;
              imu_msg.linear_acceleration.z = az;
              esp32Callback(std::make_shared<sensor_msgs::msg::Imu>(imu_msg));
          } else {
              RCLCPP_WARN(this->get_logger(), "Invalid data: %s", buffer);
          }
      }
  }
  bool is_all_zero(const std::vector<float>& data)
  {
      if (data.size() != 6)
          return true;  // invalid data treated as zero-set

      for (float value : data) {
          if (fabs(value) > tolerance_)
              return false;  // found a non-zero value
      }
      return true;  // all values are zero
  }
  void esp32Callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE){
      RCLCPP_WARN(get_logger(), "Node is not active. Ignoring incoming IMU data.");
      return;
    }
    RCLCPP_INFO(
      get_logger(),
      "Reveiced IMU data:\n"
      "Linear Acceleration: x=%.3f, y=%.3f, z=%.3f\n"
      "Angular Velocity: x=%.3f, y=%.3f, z=%.3f\n"
      "Time: sec=%u, nanosec=%u",
      msg->linear_acceleration.x,
      msg->linear_acceleration.y,
      msg->linear_acceleration.z,
      msg->angular_velocity.x,
      msg->angular_velocity.y,
      msg->angular_velocity.z,
      msg->header.stamp.sec,
      msg->header.stamp.nanosec
    );

    RCLCPP_INFO(get_logger(), "Publish data to database subscriber..." );

    publisher_->publish(*msg);
  }

  int sockfd_;
  int port_;
  int tolerance_;
  int timer_period_ms_;
  bool connection_type_; // 0 for wired, 1 for wireless
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

#ifndef TESTING_EXCLUDE_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IMULifecycleNode>(rclcpp::NodeOptions());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
#endif
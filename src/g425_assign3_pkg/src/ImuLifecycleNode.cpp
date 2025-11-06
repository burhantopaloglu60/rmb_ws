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
*/

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "lifecycle_msgs/msg/transition.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <lifecycle_msgs/msg/state.hpp>

using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using CallbackReturn = LifecycleNodeInterface::CallbackReturn;

class IMULifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  IMULifecycleNode(const rclcpp::NodeOptions & options)
  : rclcpp_lifecycle::LifecycleNode("imu_lifecycle_node", options)
  {
    RCLCPP_INFO(get_logger(), "Lifecycle node started in state: %s", this->get_current_state().label().c_str());
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Configuring IMU lifecycle node...");

    // Publisher to /imua_data so the subscriber can put the data into the database
    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu_data", 10);

    // Subscriber on topic of ESP32
    subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/esp32_topic",
      10,
      std::bind(&IMULifecycleNode::esp32Callback, this, std::placeholders::_1)
    );

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
  void esp32Callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      return;

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

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

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

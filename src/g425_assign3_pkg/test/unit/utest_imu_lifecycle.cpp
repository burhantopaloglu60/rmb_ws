#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "../src/ImuLifecycleNode.cpp"


class TestIMULifecycleNode : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }
};

// Test 1: node can be created
TEST_F(TestIMULifecycleNode, NodeCanBeCreated)
{
  auto node = std::make_shared<IMULifecycleNode>(rclcpp::NodeOptions());
  ASSERT_NE(node, nullptr);
}

// Test 2: node publishes something to /imu_data after receiving a message from /esp32_topic
TEST_F(TestIMULifecycleNode, PublishesImuDataWhenActive)
{
  auto node = std::make_shared<IMULifecycleNode>(rclcpp::NodeOptions());
  auto publisher = node->create_publisher<sensor_msgs::msg::Imu>("/esp32_topic", 10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  // Subscriber to capture output
  sensor_msgs::msg::Imu::SharedPtr received_msg = nullptr;
  auto sub = node->create_subscription<sensor_msgs::msg::Imu>(
    "/imu_data", 10,
    [&](sensor_msgs::msg::Imu::SharedPtr msg) {
      received_msg = msg;
    });

  // Manually activate the node (otherwise it won't publish)
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  // IMU test message
  sensor_msgs::msg::Imu msg;
  msg.linear_acceleration.x = 1.0550;
  msg.linear_acceleration.y = 3.2444;
  msg.linear_acceleration.z = 2.8768;
  msg.angular_velocity.x = -0.3;
  msg.angular_velocity.y = 0.6;
  msg.angular_velocity.z = 0.5;
  msg.header.stamp = rclcpp::Clock().now();

  publisher->publish(msg);

  // Run event loop for max 2 sec
  auto start = node->now();
  while (rclcpp::ok() && received_msg == nullptr &&
         (node->now() - start).seconds() < 2.0)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  EXPECT_NE(received_msg, nullptr);
  if (received_msg)
  {
    EXPECT_NEAR(received_msg->linear_acceleration.x, 1.0550, 1e-6);
    EXPECT_NEAR(received_msg->linear_acceleration.y, 3.2444, 1e-6);
    EXPECT_NEAR(received_msg->linear_acceleration.z, 2.8768, 1e-6);

    EXPECT_NEAR(received_msg->angular_velocity.x, -0.3, 1e-6);
    EXPECT_NEAR(received_msg->angular_velocity.y,  0.6, 1e-6);
    EXPECT_NEAR(received_msg->angular_velocity.z,  0.5, 1e-6);

  }
}

// Test 3: Node does NOT publish when deactivated
TEST_F(TestIMULifecycleNode, DoesNotPublishWhenDeactivated)
{
  auto node = std::make_shared<IMULifecycleNode>(rclcpp::NodeOptions());
  auto publisher = node->create_publisher<sensor_msgs::msg::Imu>("/esp32_topic", 10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  sensor_msgs::msg::Imu::SharedPtr received_msg = nullptr;
  auto sub = node->create_subscription<sensor_msgs::msg::Imu>(
    "/imu_data", 10,
    [&](sensor_msgs::msg::Imu::SharedPtr msg) {
      received_msg = msg;
    });

  // Configure and activate, then deactivate
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);

  // Post message while node is deactivated
  sensor_msgs::msg::Imu msg;
  msg.linear_acceleration.x = 2.0;
  msg.angular_velocity.z = 1.0;
  msg.header.stamp.sec = 5678;

  publisher->publish(msg);

  auto start = node->now();
  while (rclcpp::ok() && received_msg == nullptr &&
         (node->now() - start).seconds() < 2.0)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // Expect NO message (because node is deactivated)
  EXPECT_EQ(received_msg, nullptr);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

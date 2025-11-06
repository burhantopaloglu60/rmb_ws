#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <memory>
#include "../src/LifecycleManager.cpp"

class TestLifecycleManager : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr); 
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

// Test 1: Node can be instantiated
TEST_F(TestLifecycleManager, NodeCanBeCreated)
{
  auto node = std::make_shared<LifecycleManager>("test_node");
  ASSERT_NE(node, nullptr);
}

// Test 2: Callback updates `data_received_` and `last_msg_time_`
TEST_F(TestLifecycleManager, EspCallbackUpdatesData)
{
  auto node = std::make_shared<LifecycleManager>("test_node");

  sensor_msgs::msg::Imu msg;
  msg.linear_acceleration.x = 1.0;
  msg.linear_acceleration.y = 2.0;
  msg.linear_acceleration.z = 3.0;
  msg.angular_velocity.x = 0.1;
  msg.angular_velocity.y = 0.2;
  msg.angular_velocity.z = 0.3;

  rclcpp::Time before = node->now();
  node->esp_callback(std::make_shared<sensor_msgs::msg::Imu>(msg));
  rclcpp::Time after = node->now();

  EXPECT_TRUE(node->is_connection_alive(1.0));
  // last_msg_time_ should be updated between before and after
}

// Test 3: is_connection_alive returns false if no data received
TEST_F(TestLifecycleManager, ConnectionCheck)
{
  auto node = std::make_shared<LifecycleManager>("test_node");
  EXPECT_FALSE(node->is_connection_alive(0.1)); // No data yet
  // Send data to mark connection alive
  sensor_msgs::msg::Imu msg;
  node->esp_callback(std::make_shared<sensor_msgs::msg::Imu>(msg));
  EXPECT_TRUE(node->is_connection_alive(1.0));
}

// Test 4: Connection times out after inactivity
TEST_F(TestLifecycleManager, ConnectionTimeout)
{
  auto node = std::make_shared<LifecycleManager>("test_node");

  // Simulate receiving one message
  sensor_msgs::msg::Imu msg;
  node->esp_callback(std::make_shared<sensor_msgs::msg::Imu>(msg));

  // Immediately after callback, connection should be alive
  EXPECT_TRUE(node->is_connection_alive(1.0));

  // Wait longer than the timeout (e.g., 1 second)
  std::this_thread::sleep_for(std::chrono::milliseconds(1200));

  // Now, connection should be considered dead if timeout = 1.0 second
  EXPECT_FALSE(node->is_connection_alive(1.0));
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

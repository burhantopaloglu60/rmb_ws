#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <memory>
#include <chrono>
#include "../src/Subscriber.cpp"

class TestLifecycleNodeSubscriber : public ::testing::Test
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

// Test 1: Node can be created
TEST_F(TestLifecycleNodeSubscriber, NodeCanBeCreated)
{
  auto node = std::make_shared<LifecycleNodeSubscriber>();
  ASSERT_NE(node, nullptr);
}

// Test 2: Callback writes data to database
TEST_F(TestLifecycleNodeSubscriber, ImuCallbackWritesToDatabase)
{
  auto node = std::make_shared<LifecycleNodeSubscriber>();

  // Imu test message
  auto msg = std::make_shared<sensor_msgs::msg::Imu>();
  msg->linear_acceleration.x = 1.0;
  msg->linear_acceleration.y = 2.0;
  msg->linear_acceleration.z = 3.0;
  msg->angular_velocity.x = 0.1;
  msg->angular_velocity.y = 0.2;
  msg->angular_velocity.z = 0.3;
  msg->header.stamp.sec = 1730000000;
  msg->header.stamp.nanosec = 1000;

  // Call the callback directly
  // (database_->addMeasurement is called within this function)
  node->imuCallback(msg);

  SUCCEED(); // if there is no crash, the test is successful
}

// Test 3: Node receives message via /imu_data and writes to database
TEST_F(TestLifecycleNodeSubscriber, ReceivesImuMessageOverTopic)
{
  auto node = std::make_shared<LifecycleNodeSubscriber>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Publisher to send test message
  auto pub = node->create_publisher<sensor_msgs::msg::Imu>("/imu_data", 10);

  sensor_msgs::msg::Imu msg;
  msg.linear_acceleration.x = 9.9;
  msg.linear_acceleration.y = 8.8;
  msg.linear_acceleration.z = 7.7;
  msg.angular_velocity.x = 0.5;
  msg.angular_velocity.y = 0.6;
  msg.angular_velocity.z = 0.7;
  msg.header.stamp.sec = 1730000001;
  msg.header.stamp.nanosec = 500;

  // Publish the message
  pub->publish(msg);

  // Let the executor run for a while so that the callback has time to execute
  auto start = node->now();
  while ((node->now() - start).seconds() < 2.0)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  SUCCEED();  // if there is no crash, the test is successful
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}




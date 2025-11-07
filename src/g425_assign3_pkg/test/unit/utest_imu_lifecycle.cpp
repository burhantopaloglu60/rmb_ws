#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "../src/ImuLifecycleNode.cpp"
#include <arpa/inet.h>
#include <thread>
#include <chrono>
class MockWirelessIMUNode : public IMULifecycleNode
{
public:
  using IMULifecycleNode::IMULifecycleNode;

  void inject_test_udp_data(const std::string &data)
  {
    // Simulate receiving data as if it came in via UDP
    char buffer[256];
    strncpy(buffer, data.c_str(), sizeof(buffer));
    buffer[sizeof(buffer) - 1] = '\0';

    float gx, gy, gz, ax, ay, az;
    if (sscanf(buffer, "%f,%f,%f,%f,%f,%f", &gx, &gy, &gz, &ax, &ay, &az) == 6)
    {
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
    }
  }
};


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

// Test 2: Wired Mode — publishes to /imu_data after receiving data on /imu_data_esp
TEST_F(TestIMULifecycleNode, PublishesImuDataInWiredMode)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"connection_type", false}});
  auto node = std::make_shared<IMULifecycleNode>(options);

  auto publisher = node->create_publisher<sensor_msgs::msg::Imu>("imu_data_esp", 10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  sensor_msgs::msg::Imu::SharedPtr received_msg = nullptr;

  // Subscriber to capture output
  auto sub = node->create_subscription<sensor_msgs::msg::Imu>(
    "imu_data", 10,
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
    EXPECT_NEAR(received_msg->angular_velocity.y, 0.6, 1e-6);
    EXPECT_NEAR(received_msg->angular_velocity.z, 0.5, 1e-6);
  }
}

// Test 3: Wireless Mode — publishes after UDP packet received
TEST_F(TestIMULifecycleNode, PublishesImuDataInWirelessMode)
{
  // Wireless mode
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"connection_type", true}});
  auto node = std::make_shared<MockWirelessIMUNode>(options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  sensor_msgs::msg::Imu::SharedPtr received_msg = nullptr;
  auto sub = node->create_subscription<sensor_msgs::msg::Imu>(
    "imu_data", 10,
    [&](sensor_msgs::msg::Imu::SharedPtr msg) {
      received_msg = msg;
    });

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  // Test UDP message
  node->inject_test_udp_data("0.1,0.2,0.3,9.81,0.0,0.0");

  // Allow node to process
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
    EXPECT_NEAR(received_msg->angular_velocity.x, 0.1, 1e-6);
    EXPECT_NEAR(received_msg->angular_velocity.y, 0.2, 1e-6);
    EXPECT_NEAR(received_msg->angular_velocity.z, 0.3, 1e-6);
    EXPECT_NEAR(received_msg->linear_acceleration.x, 9.81, 1e-6);
    EXPECT_NEAR(received_msg->linear_acceleration.y, 0.0, 1e-6);
    EXPECT_NEAR(received_msg->linear_acceleration.z, 0.0, 1e-6);
  }
}

// Test 4: Does NOT publish when deactivated (any mode)
TEST_F(TestIMULifecycleNode, DoesNotPublishWhenDeactivated)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"connection_type", false}});
  auto node = std::make_shared<IMULifecycleNode>(options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  sensor_msgs::msg::Imu::SharedPtr received_msg = nullptr;
  auto sub = node->create_subscription<sensor_msgs::msg::Imu>(
    "imu_data", 10,
    [&](sensor_msgs::msg::Imu::SharedPtr msg) { received_msg = msg; });

  // Configure and activate, then deactivate
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);

  // Publish message while deactivated
  auto pub = node->create_publisher<sensor_msgs::msg::Imu>("imu_data_esp", 10);
  sensor_msgs::msg::Imu msg;
  msg.linear_acceleration.x = 1.0;
  msg.angular_velocity.z = 1.0;
  pub->publish(msg);

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

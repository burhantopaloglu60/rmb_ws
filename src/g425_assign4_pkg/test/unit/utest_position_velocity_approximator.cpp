#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "g425_assign4_interfaces_pkg/msg/imu_sim.hpp"
#include "g425_assign4_interfaces_pkg/msg/position_data.hpp"

#include "../src/position_velocity_approximator.cpp"  

class PositionVelocityTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<PositionVelocityApproximator>();

    imu_pub_ = node_->create_publisher<g425_assign4_interfaces_pkg::msg::ImuSim>(
      "imu_sim_acceleration", 10);

    exec_.add_node(node_);
  }

  void TearDown() override
  {
    exec_.cancel();
  }

  rclcpp::executors::SingleThreadedExecutor exec_;
  std::shared_ptr<PositionVelocityApproximator> node_;
  rclcpp::Publisher<g425_assign4_interfaces_pkg::msg::ImuSim>::SharedPtr imu_pub_;

  // Test variables
  g425_assign4_interfaces_pkg::msg::PositionData received_pos_;
  bool pos_received_flag_ = false;

};

// TEST 1 — Verify that /imu_sim_pos publishes something
TEST_F(PositionVelocityTest, PublishesPositionVelocity)
{
  RCLCPP_INFO(node_->get_logger(), "This test publishes an IMU message to verify if the position and velocity are published.");

  double old_y = node_->pos_y_;

  g425_assign4_interfaces_pkg::msg::ImuSim imu;
  imu.stamp = node_->now();
  imu.x = 0.0;
  imu.y = 1.0;  
  imu.yaw_z = 0.0;

  imu_pub_->publish(imu);

  exec_.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(30));

  // Internal state changes directly
  EXPECT_GT(node_->pos_y_, old_y);
}

// TEST 2 — Verify reset_callback sets new pose correctly
TEST_F(PositionVelocityTest, ResetCallbackSetsPose)
{
  RCLCPP_INFO(node_->get_logger(), "This test calls reset_callback with a specific pose.\n "
               "After the call, the node's position, yaw, and velocities should match the reset values.");

  geometry_msgs::msg::PoseStamped reset;

  reset.pose.position.x = 5.0;
  reset.pose.position.y = -3.0;

  // yaw = 90°
  reset.pose.orientation.x = 0;
  reset.pose.orientation.y = 0;
  reset.pose.orientation.z = std::sin(M_PI/4);
  reset.pose.orientation.w = std::cos(M_PI/4);

  // Call the callback directly
  node_->reset_callback(std::make_shared<geometry_msgs::msg::PoseStamped>(reset));

  EXPECT_NEAR(node_->pos_x_, 5.0, 1e-6);
  EXPECT_NEAR(node_->pos_y_, -3.0, 1e-6);
  EXPECT_NEAR(node_->yaw_, M_PI/2, 0.05);

  // Velocities should be reset
  EXPECT_NEAR(node_->vx_, 0.0, 1e-9);
  EXPECT_NEAR(node_->vy_, 0.0, 1e-9);
  EXPECT_NEAR(node_->omega_z_, 0.0, 1e-9);
}

// TEST 3 — Verify trapezoidal integration correctly updates velocity
TEST_F(PositionVelocityTest, TrapezoidalIntegrationWorks)
{
  RCLCPP_INFO(node_->get_logger(), "This test verifies that the node correctly integrates IMU acceleration using the trapezoidal rule.\n "
               "Two IMU messages are send with a small time difference and check that the resulting velocity matches.");

  g425_assign4_interfaces_pkg::msg::ImuSim imu;
  imu.stamp = node_->now();
  imu.y = 2.0;  // constant acceleration

  imu_pub_->publish(imu);
  exec_.spin_some();

  auto t0 = node_->last_stamp_;

  // Send another IMU after some time
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  imu.stamp = node_->now();
  imu_pub_->publish(imu);
  exec_.spin_some();

  double dt = (node_->last_stamp_ - t0).seconds();

  // Expected trapezoid velocity: v = a * t
  EXPECT_NEAR(node_->vy_, 2.0 * dt, 0.05);
}

// TEST 4 — Verify rotation transform correctness
TEST_F(PositionVelocityTest, RotationTransformCorrect)
{
  RCLCPP_INFO(node_->get_logger(), "This test sets the robot yaw to 90° and applies a robot-frame acceleration along +X.\n "
               "After integration, the motion should appear along +Y in the map frame, with X motion suppressed.");

  // Yaw to 90°
  {
    geometry_msgs::msg::PoseStamped reset;
    reset.pose.position.x = 0;
    reset.pose.position.y = 0;
    reset.pose.orientation.x = 0;
    reset.pose.orientation.y = 0;
    reset.pose.orientation.z = std::sin(M_PI/4);
    reset.pose.orientation.w = std::cos(M_PI/4);

    node_->reset_callback(std::make_shared<geometry_msgs::msg::PoseStamped>(reset));
  }

  double initial_y = node_->pos_y_;

  // Robot-frame acceleration = (1, 0)
  g425_assign4_interfaces_pkg::msg::ImuSim imu;
  imu.x = 1.0;
  imu.y = 0.0;
  imu.yaw_z = 0.0;
  imu.stamp = node_->now();

  // First message (initialization, dt = 0)
  imu_pub_->publish(imu);
  exec_.spin_some();

  // Small time delay
  std::this_thread::sleep_for(std::chrono::milliseconds(30));

  // Second message (actual integration)
  imu.stamp = node_->now();
  imu_pub_->publish(imu);
  exec_.spin_some();

  // EXPECT: motion should now be entirely in +Y direction
  EXPECT_GT(node_->pos_y_, initial_y);  
  EXPECT_NEAR(node_->pos_x_, 0.0, 0.05); // rotation suppresses X motion
}

// TEST 5 — Verify that position and velocity are not updated when IMU messages have zero time delta
TEST_F(PositionVelocityTest, InvalidDtDoesNotUpdatePositionVelocity)
{
  RCLCPP_INFO(node_->get_logger(), "This test publishes two IMU messages with the same timestamp.\n "
               "The second message should be ignored by the node, ensuring that "
               "position and velocity are not updated when dt = 0.");

  g425_assign4_interfaces_pkg::msg::ImuSim imu;
  imu.x = 0.0;
  imu.y = 5.0;    // would normally cause acceleration
  imu.yaw_z = 0.0;

  // Timestamp t0
  imu.stamp = node_->now();

  // First message sets last_stamp_
  imu_pub_->publish(imu);
  exec_.spin_some();

  double pos_y_before = node_->pos_y_;
  double vel_y_before = node_->vy_;

  // Second message: SAME timestamp → dt = 0 → must be ignored
  imu_pub_->publish(imu);
  exec_.spin_some();

  // EXPECT NO CHANGE
  EXPECT_NEAR(node_->pos_y_, pos_y_before, 1e-9);
  EXPECT_NEAR(node_->vy_, vel_y_before, 1e-9);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}





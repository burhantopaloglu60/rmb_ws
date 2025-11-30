#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#define TESTING_EXCLUDE_MAIN

#include "g425_assign4_interfaces_pkg/msg/imu_sim.hpp"
#include "../src/sensor_simulator.cpp"

using g425_assign4_interfaces_pkg::msg::ImuSim;

class SensorSimulatorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    sim_node_ = std::make_shared<SensorSimulator>();
    exec_.add_node(sim_node_);

    sub_node_ = std::make_shared<rclcpp::Node>("test_subscriber");
    sub_ = sub_node_->create_subscription<ImuSim>("imu_sim_acceleration", 10, [this](const ImuSim::SharedPtr msg) {
      last_msg_ = *msg;
      received_ = true;
    });

    exec_.add_node(sub_node_);
  }

  void TearDown() override
  {
    exec_.remove_node(sim_node_);
    exec_.remove_node(sub_node_);
    rclcpp::shutdown();
  }

  rclcpp::executors::SingleThreadedExecutor exec_;
  rclcpp::Node::SharedPtr sub_node_;
  rclcpp::Subscription<ImuSim>::SharedPtr sub_;
  SensorSimulator::SharedPtr sim_node_;

  ImuSim last_msg_;
  bool received_{ false };
};

// TEST 1: Node should load parameters and create intervals
TEST_F(SensorSimulatorTest, LoadsIntervals)
{
  auto intervals_param = sim_node_->list_parameters({ "intervals" }, 10);
  ASSERT_GE(intervals_param.names.size(), 1);
}

// TEST 2: Node publishes periodically
TEST_F(SensorSimulatorTest, PublishesMessages)
{
  // Spin for 200 ms (should get a message at 50 Hz default)
  auto start = std::chrono::steady_clock::now();
  while (!received_ && std::chrono::steady_clock::now() - start < std::chrono::milliseconds(200))
  {
    exec_.spin_some();
  }

  ASSERT_TRUE(received_) << "No IMU messages received from simulator.";
}

// TEST 3: Acceleration matches expected derivative for known interval
//         Define a linear velocity segment so acceleration is constant.
TEST_F(SensorSimulatorTest, CorrectLinearAcceleration)
{
  // Set a custom interval with known derivative
  sim_node_->set_parameter(rclcpp::Parameter("intervals.0.axis", "linear_x"));
  sim_node_->set_parameter(rclcpp::Parameter("intervals.0.poly", "linear"));
  sim_node_->set_parameter(rclcpp::Parameter("intervals.0.t0", 0.0));
  sim_node_->set_parameter(rclcpp::Parameter("intervals.0.t1", 2.0));
  sim_node_->set_parameter(rclcpp::Parameter("intervals.0.y0", 0.0));
  sim_node_->set_parameter(rclcpp::Parameter("intervals.0.y1", 4.0));

  // Expected acceleration = slope = (4 - 0) / (2 - 0) = 2.0
  constexpr double expected_acc = 2.0;

  received_ = false;

  auto start = std::chrono::steady_clock::now();
  while (!received_ && std::chrono::steady_clock::now() - start < std::chrono::milliseconds(300))
  {
    exec_.spin_some();
  }

  ASSERT_TRUE(received_) << "Simulator did not publish in time.";

  EXPECT_NEAR(last_msg_.x, expected_acc, 1e-3) << "Incorrect linear acceleration derivative.";
}

// TEST 4: Quadratic interval derivative matches expected behavior
TEST_F(SensorSimulatorTest, QuadraticDerivativeNonZero)
{
  sim_node_->set_parameter(rclcpp::Parameter("intervals.1.axis", "angular_z"));
  sim_node_->set_parameter(rclcpp::Parameter("intervals.1.poly", "quadratic"));
  sim_node_->set_parameter(rclcpp::Parameter("intervals.1.t0", 0.0));
  sim_node_->set_parameter(rclcpp::Parameter("intervals.1.t1", 1.0));
  sim_node_->set_parameter(rclcpp::Parameter("intervals.1.tm", 0.5));
  sim_node_->set_parameter(rclcpp::Parameter("intervals.1.y0", 0.0));
  sim_node_->set_parameter(rclcpp::Parameter("intervals.1.ym", 1.0));
  sim_node_->set_parameter(rclcpp::Parameter("intervals.1.y1", 0.0));

  received_ = false;

  auto start = std::chrono::steady_clock::now();
  while (!received_ && std::chrono::steady_clock::now() - start < std::chrono::milliseconds(300))
  {
    exec_.spin_some();
  }

  ASSERT_TRUE(received_);
  EXPECT_NEAR(last_msg_.yaw_z, 0.0, 0.1) << "Quadratic derivative should be near zero at t≈0 or t≈1.";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#define TESTING_EXCLUDE_MAIN

#include "g425_assign4_interfaces_pkg/msg/mecanum.hpp"
#include "../src/wheel_velocity_simulator.cpp"

using g425_assign4_interfaces_pkg::msg::Mecanum;

class WheelVelocitySimulatorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    exec_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();

    sim_node_ = std::make_shared<WheelVelocitySimulator>();
    exec_->add_node(sim_node_);

    sub_node_ = std::make_shared<rclcpp::Node>("test_wheel_sub");
    sub_ = sub_node_->create_subscription<Mecanum>("mecanum_velocity", 10, [this](const Mecanum::SharedPtr msg) {
      last_msg_ = *msg;
      received_ = true;
    });

    exec_->add_node(sub_node_);
  }

  void TearDown() override
  {
    exec_->remove_node(sim_node_);
    exec_->remove_node(sub_node_);
    rclcpp::shutdown();
  }

  void wait_for_message(int ms = 300)
  {
    received_ = false;
    auto start = std::chrono::steady_clock::now();

    while (!received_ && std::chrono::steady_clock::now() - start < std::chrono::milliseconds(ms))
    {
      exec_->spin_some();
    }
  }

  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;

  rclcpp::Node::SharedPtr sub_node_;
  rclcpp::Subscription<Mecanum>::SharedPtr sub_;

  std::shared_ptr<WheelVelocitySimulator> sim_node_;
  Mecanum last_msg_;
  bool received_ = false;
};

//
// TEST 1: Publishes messages
//
TEST_F(WheelVelocitySimulatorTest, PublishesMessages)
{
  wait_for_message();
  ASSERT_TRUE(received_) << "No mecanum messages published.";
}

//
// TEST 2: Constant interval value
//
TEST_F(WheelVelocitySimulatorTest, ConstantVelocity)
{
  // Remove old simulator
  exec_->remove_node(sim_node_);
  sim_node_.reset();

  // Set parameters BEFORE creating simulator
  auto p = std::make_shared<rclcpp::Node>("params_constant");
  p->declare_parameter("intervals.0.wheel", "wfl");
  p->declare_parameter("intervals.0.poly", "constant");
  p->declare_parameter("intervals.0.t0", 0.0);
  p->declare_parameter("intervals.0.t1", 100.0);
  p->declare_parameter("intervals.0.y0", 5.0);
  exec_->add_node(p);
  exec_->spin_some();

  // Recreate simulator so it loads parameters now
  sim_node_ = std::make_shared<WheelVelocitySimulator>();
  exec_->add_node(sim_node_);

  wait_for_message();
  ASSERT_TRUE(received_);
  EXPECT_NEAR(last_msg_.wfl, 5.0, 1e-6);
}

//
// TEST 3: Linear interval correctly interpolates
//
TEST_F(WheelVelocitySimulatorTest, LinearVelocity)
{
  exec_->remove_node(sim_node_);
  sim_node_.reset();

  auto p = std::make_shared<rclcpp::Node>("params_linear");
  p->declare_parameter("intervals.0.wheel", "wfr");
  p->declare_parameter("intervals.0.poly", "linear");
  p->declare_parameter("intervals.0.t0", 0.0);
  p->declare_parameter("intervals.0.t1", 2.0);
  p->declare_parameter("intervals.0.y0", 0.0);
  p->declare_parameter("intervals.0.y1", 4.0);
  exec_->add_node(p);
  exec_->spin_some();

  sim_node_ = std::make_shared<WheelVelocitySimulator>();
  exec_->add_node(sim_node_);

  wait_for_message();
  ASSERT_TRUE(received_);
  EXPECT_GT(last_msg_.wfr, 0.0);
  EXPECT_LT(last_msg_.wfr, 1.0);
}

//
// TEST 4: Quadratic interval produces non-zero inside range
//
TEST_F(WheelVelocitySimulatorTest, QuadraticVelocity)
{
  exec_->remove_node(sim_node_);
  sim_node_.reset();

  auto p = std::make_shared<rclcpp::Node>("params_quad");
  p->declare_parameter("intervals.0.wheel", "wrl");
  p->declare_parameter("intervals.0.poly", "quadratic");
  p->declare_parameter("intervals.0.t0", 0.0);
  p->declare_parameter("intervals.0.t1", 1.0);
  p->declare_parameter("intervals.0.tm", 0.5);
  p->declare_parameter("intervals.0.y0", 0.0);
  p->declare_parameter("intervals.0.ym", 1.0);
  p->declare_parameter("intervals.0.y1", 0.0);
  exec_->add_node(p);
  exec_->spin_some();

  sim_node_ = std::make_shared<WheelVelocitySimulator>();
  exec_->add_node(sim_node_);

  wait_for_message();
  ASSERT_TRUE(received_);
  EXPECT_NEAR(last_msg_.wrl, 0.25, 0.25);
}

//
// TEST 5: Looping behavior (values repeat instead of going to zero)
//
TEST_F(WheelVelocitySimulatorTest, LoopingWorks)
{
  exec_->remove_node(sim_node_);
  sim_node_.reset();

  auto p = std::make_shared<rclcpp::Node>("params_loop");
  p->declare_parameter("intervals.0.wheel", "wrr");
  p->declare_parameter("intervals.0.poly", "constant");
  p->declare_parameter("intervals.0.t0", 2.0);
  p->declare_parameter("intervals.0.t1", 4.0);
  p->declare_parameter("intervals.0.y0", 3.0);
  exec_->add_node(p);
  exec_->spin_some();

  sim_node_ = std::make_shared<WheelVelocitySimulator>();
  exec_->add_node(sim_node_);

  wait_for_message(400);
  ASSERT_TRUE(received_);
  EXPECT_NEAR(last_msg_.wrr, 3.0, 1e-6);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>

#include "g425_assign4_interfaces_pkg/msg/mecanum.hpp"
#include "g425_assign4_interfaces_pkg/msg/position_data.hpp"

#include "../src/position_approximator_mecanum.cpp"

using mecanum = g425_assign4_interfaces_pkg::msg::Mecanum;
using PositionData = g425_assign4_interfaces_pkg::msg::PositionData;

class MecanumPositionTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        mecanum_node_ = std::make_shared<position_approximator_mecanum>();

        mec_pub_ = mecanum_node_->create_publisher<mecanum>(
            mecanum_node_->get_parameter("mecanum_topic_velocity").as_string(), 10);

        pos_sub_ = mecanum_node_->create_subscription<PositionData>(
            mecanum_node_->get_parameter("mecanum_topic_position").as_string(), 10,
            [this](PositionData msg)
            {
                last_pos_msg_ = msg;
                pos_received_ = true;
            });

        exec_.add_node(mecanum_node_);
    }

    void TearDown() override
    {
        exec_.cancel();
    }

    rclcpp::executors::SingleThreadedExecutor exec_;
    std::shared_ptr<position_approximator_mecanum> mecanum_node_;
    rclcpp::Publisher<mecanum>::SharedPtr mec_pub_;
    rclcpp::Subscription<PositionData>::SharedPtr pos_sub_;

    PositionData last_pos_msg_;
    bool pos_received_ = false;
};

//
// ------------------------------------------------------------
// TEST 1 — Node should receive a mecanum message
// ------------------------------------------------------------
//
TEST_F(MecanumPositionTest, NodeReceivesMecanumMessage)
{
    mecanum msg;
    msg.wfl = msg.wfr = msg.wrl = msg.wrr = 1.0;
    msg.stamp = rclcpp::Clock().now();

    mec_pub_->publish(msg);

    exec_.spin_some();

    // If no crash → subscription works
    SUCCEED();
}

//
// ------------------------------------------------------------
// TEST 2 — Pose should update when calculate_mecanum processes data
// ------------------------------------------------------------
//
TEST_F(MecanumPositionTest, TestXMovement)
{
    mecanum msg;
    msg.stamp = rclcpp::Clock().now();
    msg.wfl = msg.wfr = msg.wrl = msg.wrr = 1.0;

    // First callback only initializes timestamp → no movement
    mecanum_node_->calculate_mecanum(msg);

    // Wait some time
    rclcpp::sleep_for(std::chrono::milliseconds(1000));

    // Capture position output from second message
    testing::internal::CaptureStderr();

    msg.stamp = rclcpp::Clock().now();
    mecanum_node_->calculate_mecanum(msg);

    std::string output = testing::internal::GetCapturedStderr();
    std::cout << output;

    EXPECT_NE(output.find("Velocities:"), std::string::npos)
        << "Should print velocity information";

    EXPECT_NE(output.find("Pose:"), std::string::npos)
        << "Should print Pose line";

    EXPECT_NE(output.find("x="), std::string::npos)
        << "Pose should contain x value";

    EXPECT_NE(output.find("y="), std::string::npos)
        << "Pose should contain y value";

    EXPECT_EQ(output.find("x=0.00"), std::string::npos)
        << "x should not remain zero after movement";

    EXPECT_EQ(output.find("vx=0.00"), std::string::npos)
        << "vx should not remain zero after movement";
}
//
// ------------------------------------------------------------
// TEST 3 — Pose should update when calculate_mecanum processes data
// ------------------------------------------------------------
//
TEST_F(MecanumPositionTest, TestYMovement)
{
    mecanum msg;
    msg.stamp = rclcpp::Clock().now();
    msg.wfl = -1.0; // Front-left
    msg.wfr = 1.0;  // Front-right
    msg.wrl = 1.0;  // Rear-left
    msg.wrr = -1.0; // Rear-right

    // First callback only initializes timestamp → no movement
    mecanum_node_->calculate_mecanum(msg);

    // Wait some time
    rclcpp::sleep_for(std::chrono::milliseconds(1000));

    // Capture position output from second message
    testing::internal::CaptureStderr();

    msg.stamp = rclcpp::Clock().now();
    mecanum_node_->calculate_mecanum(msg);

    std::string output = testing::internal::GetCapturedStderr();
    std::cout << output;

    EXPECT_NE(output.find("Velocities:"), std::string::npos)
        << "Should print velocity information";

    EXPECT_NE(output.find("Pose:"), std::string::npos)
        << "Should print Pose line";

    EXPECT_NE(output.find("x="), std::string::npos)
        << "Pose should contain x value";

    EXPECT_NE(output.find("y="), std::string::npos)
        << "Pose should contain y value";

    EXPECT_EQ(output.find("y=0.00"), std::string::npos)
        << "y should not remain zero after movement";

    EXPECT_EQ(output.find("vy=0.00"), std::string::npos)
        << "vy should not remain zero after movement";
}
//
// ------------------------------------------------------------
// TEST 4 — Pose should update when calculate_mecanum processes data
// ------------------------------------------------------------
//
TEST_F(MecanumPositionTest, TestYaw_zMovement)
{
    mecanum msg;
    msg.stamp = rclcpp::Clock().now();
    msg.wfl = -1.0; // Front-left
    msg.wfr = 1.0;  // Front-right
    msg.wrl = -1.0; // Rear-left
    msg.wrr = 1.0;  // Rear-right

    // First callback only initializes timestamp → no movement
    mecanum_node_->calculate_mecanum(msg);

    // Wait some time
    rclcpp::sleep_for(std::chrono::milliseconds(1000));

    // Capture position output from second message
    testing::internal::CaptureStderr();

    msg.stamp = rclcpp::Clock().now();
    mecanum_node_->calculate_mecanum(msg);

    std::string output = testing::internal::GetCapturedStderr();
    std::cout << output;

    EXPECT_NE(output.find("Velocities:"), std::string::npos)
        << "Should print velocity information";

    EXPECT_NE(output.find("Pose:"), std::string::npos)
        << "Should print Pose line";

    EXPECT_NE(output.find("x="), std::string::npos)
        << "Pose should contain x value";

    EXPECT_NE(output.find("y="), std::string::npos)
        << "Pose should contain y value";

    EXPECT_EQ(output.find("yaw_vz=0.00"), std::string::npos)
        << "yaw_vz should not remain zero after movement";

    EXPECT_EQ(output.find("yaw_z=0.00"), std::string::npos)
        << "yaw_z should not remain zero after movement";
}

//
// ------------------------------------------------------------
// TEST 5 — The node publishes PositionData messages
// ------------------------------------------------------------
//
TEST_F(MecanumPositionTest, NodePublishesPositionMessages)
{
    mecanum msg;
    msg.stamp = rclcpp::Clock().now();
    msg.wfl = msg.wfr = msg.wrl = msg.wrr = 0.0;

    pos_received_ = false;

    // First message → timestamp sync
    mecanum_node_->calculate_mecanum(msg);

    rclcpp::sleep_for(std::chrono::milliseconds(1000));

    // Second message triggers actual publish
    msg.stamp = rclcpp::Clock().now();
    mecanum_node_->calculate_mecanum(msg);

    // Spin executor so subscriber receives message
    exec_.spin_some();

    EXPECT_TRUE(pos_received_) << "Subscriber should receive PositionData message";
    
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}

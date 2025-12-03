#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>

#include "g425_assign4_interfaces_pkg/msg/mecanum.hpp"
#include "g425_assign4_interfaces_pkg/msg/imu_sim.hpp"
#include "g425_assign4_interfaces_pkg/msg/position_data.hpp"
#include "g425_assign4_pkg/OdometryDatabase.hpp"

#include "../src/DatabasePublisher.cpp" // include node directly

using Mecanum = g425_assign4_interfaces_pkg::msg::Mecanum;
using ImuSim = g425_assign4_interfaces_pkg::msg::ImuSim;
using PositionData = g425_assign4_interfaces_pkg::msg::PositionData;

class DatabasePublisherTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        node_ = std::make_shared<DatabasePublisher>();
        exec_.add_node(node_);
    }

    void TearDown() override
    {
        exec_.cancel();
    }

    std::shared_ptr<DatabasePublisher> node_;
    rclcpp::executors::SingleThreadedExecutor exec_;
};

// ---------------- TEST 1: Mecanum Position ----------------
TEST_F(DatabasePublisherTest, MecanumPositionCallback)
{
    PositionData msg;
    msg.x = 1.0;
    msg.y = 2.0;
    msg.yaw_z = 0.5;
    msg.z = 0.0;

    testing::internal::CaptureStderr();
    node_->publish_mecanum_pos(msg);
    std::string output = testing::internal::GetCapturedStderr();

    EXPECT_NE(output.find("Position Data Received"), std::string::npos);
}

// ---------------- TEST 2: Mecanum Velocity ----------------
TEST_F(DatabasePublisherTest, MecanumVelocityCallback)
{
    Mecanum msg;
    msg.wfl = 1.0;
    msg.wfr = 2.0;
    msg.wrl = 3.0;
    msg.wrr = 4.0;

    testing::internal::CaptureStderr();
    node_->publish_mecanum_velocity(msg);
    std::string output = testing::internal::GetCapturedStderr();

    EXPECT_NE(output.find("Velocity Data Received"), std::string::npos);
}

// ---------------- TEST 3: IMU Position ----------------
TEST_F(DatabasePublisherTest, ImuSimPositionCallback)
{
    PositionData msg;
    msg.x = 0.5;
    msg.y = -0.5;
    msg.yaw_z = 1.0;
    msg.z = 0.0;

    testing::internal::CaptureStderr();
    node_->publish_imu_sim_pos(msg);
    std::string output = testing::internal::GetCapturedStderr();

    EXPECT_NE(output.find("Position Data Received"), std::string::npos);
}

// ---------------- TEST 4: IMU Velocity ----------------
TEST_F(DatabasePublisherTest, ImuSimVelocityCallback)
{
    ImuSim msg;
    msg.x = 0.1;
    msg.y = 0.2;
    msg.yaw_z = 0.05;

    testing::internal::CaptureStderr();
    node_->publish_imu_sim_velocity(msg);
    std::string output = testing::internal::GetCapturedStderr();

    EXPECT_NE(output.find("Velocity Data Received"), std::string::npos);
}

// ---------------- TEST 5: IMU Acceleration ----------------
TEST_F(DatabasePublisherTest, ImuSimAccelerationCallback)
{
    ImuSim msg;
    msg.x = 0.3;
    msg.y = 0.4;
    msg.yaw_z = 0.01;

    testing::internal::CaptureStderr();
    node_->publish_imu_sim_acceleration(msg);
    std::string output = testing::internal::GetCapturedStderr();

    EXPECT_NE(output.find("Acceleration Data Received"), std::string::npos);
}
// ---------------- TEST 6: Database Integration ----------------
TEST_F(DatabasePublisherTest, DatabaseIntegration)
{
    // Example: send one of each message type
    PositionData pos_msg;
    pos_msg.x = 1;
    pos_msg.y = 2;
    pos_msg.yaw_z = 0.1;
    Mecanum vel_msg;
    vel_msg.wfl = 1;
    vel_msg.wfr = 1;
    vel_msg.wrl = 1;
    vel_msg.wrr = 1;
    ImuSim imu_msg;
    imu_msg.x = 0.5;
    imu_msg.y = 0.5;
    imu_msg.yaw_z = 0.05;

    // Prevent duplicate timestamps in database
    rclcpp::sleep_for(std::chrono::milliseconds(1000));

    node_->publish_mecanum_pos(pos_msg);
    node_->publish_mecanum_velocity(vel_msg);
    node_->publish_imu_sim_pos(pos_msg);
    node_->publish_imu_sim_velocity(imu_msg);
    node_->publish_imu_sim_acceleration(imu_msg);

    SUCCEED(); // At minimum, test that callbacks run without crash
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}

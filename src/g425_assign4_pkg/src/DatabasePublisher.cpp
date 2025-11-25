/*
Node description: what is the node doing , what are the node objects used

*/

/*
Software changes (one line by change):
(1) 25.11.2025 created by Rik van Velzen
*/
#include <rclcpp/rclcpp.hpp>
#include "g425_assign4_interfaces_pkg/msg/mecanum.hpp"
#include "g425_assign4_interfaces_pkg/msg/imu_sim.hpp"
#include "g425_assign4_interfaces_pkg/msg/position_data.hpp"
#include "g425_assign4_pkg/OdometryDatabase.hpp"

using namespace std::placeholders;
using PositionData = g425_assign4_interfaces_pkg::msg::PositionData;
using Mecanum = g425_assign4_interfaces_pkg::msg::Mecanum;
using ImuSim = g425_assign4_interfaces_pkg::msg::ImuSim;

class DatabasePublisher : public rclcpp::Node
{
public:
    DatabasePublisher() : Node("DatabasePublisher_node")
    {
        mecanum_sub_pos_ = this->create_subscription<PositionData>(
        "mecanum_pos", 10,
        std::bind(&DatabasePublisher::publish_mecanum_pos, this, _1));

        mecanum_sub_velocity_ = this->create_subscription<Mecanum>(
        "mecanum_velocity", 10,
        std::bind(&DatabasePublisher::publish_mecanum_velocity, this, _1));

        imu_sim_sub_pos_ = this->create_subscription<PositionData>(
        "imu_sim_pos", 10,
        std::bind(&DatabasePublisher::publish_imu_sim_pos, this, _1));

        imu_sim_sub_velocity_ = this->create_subscription<ImuSim>(
        "imu_sim_velocity", 10,
        std::bind(&DatabasePublisher::publish_imu_sim_velocity, this, _1));

        imu_sim_sub_acceleration_ = this->create_subscription<ImuSim>(
        "imu_sim_acceleration", 10,
        std::bind(&DatabasePublisher::publish_imu_sim_acceleration, this, _1));
        RCLCPP_INFO(this->get_logger(), "DatabasePublisher_node active.");
    }
private:
    void publish_imu_sim_pos(const PositionData &msg)
    {
        DBT_Positions positions;
        positions.x = msg.x;
        positions.y = msg.y;
        positions.z = 0.0;
        positions.yaw_z = msg.yaw_z;
        db.addPositionImuSim(positions);

        RCLCPP_INFO(this->get_logger(),
                    "Position Data Received: x=%.2f m, y=%.2f m, yaw_z=%.2f rad",
                    msg.x, msg.y, msg.yaw_z);
        
    }
    void publish_mecanum_pos(const PositionData &msg)
    {
        DBT_Positions positions;
        positions.x = msg.x;
        positions.y = msg.y;
        positions.z = 0.0;
        positions.yaw_z = msg.yaw_z;
        db.addPositionmecanum(positions);

        RCLCPP_INFO(this->get_logger(),
                    "Position Data Received: x=%.2f m, y=%.2f m, yaw_z=%.2f rad",
                    msg.x, msg.y, msg.yaw_z);
        
    }
        void publish_mecanum_velocity(const Mecanum &msg)
    {
        DBT_Mecanum measurements;
        measurements.wfl = msg.wfl;
        measurements.wfr = msg.wfr;
        measurements.wrl = msg.wrl;
        measurements.wrr = msg.wrr;
        db.addvelocitymecanum(measurements);

        RCLCPP_INFO(this->get_logger(),
                    "Velocity Data Received: wfl=%.2f, wfr=%.2f, wrl=%.2f, wrr=%.2f",
                    msg.wfl, msg.wfr, msg.wrl, msg.wrr);
        
    }
        void publish_imu_sim_velocity(const ImuSim &msg)
    {
        DBT_Measurement measurements;
        measurements.linear_accel_x = msg.x;
        measurements.linear_accel_y = msg.y;
        measurements.linear_accel_z = 0.0;
        measurements.angular_velocity_z = msg.yaw_z;
        db.addvelocityImuSim(measurements);

        RCLCPP_INFO(this->get_logger(),
                    "Velocity Data Received: x=%.2f m, y=%.2f m, yaw_z=%.2f rad",
                    msg.x, msg.y, msg.yaw_z);
        
    }
        void publish_imu_sim_acceleration(const ImuSim &msg)
    {
        DBT_Measurement measurements;
        measurements.linear_accel_x = msg.x;
        measurements.linear_accel_y = msg.y;
        measurements.linear_accel_z = 0.0;
        measurements.angular_velocity_z = msg.yaw_z;
        db.addaccelerationImuSim(measurements);

        RCLCPP_INFO(this->get_logger(),
                    "Acceleration Data Received: x=%.2f m, y=%.2f m, yaw_z=%.2f rad",
                    msg.x, msg.y, msg.yaw_z);
        
    }

    rclcpp::Subscription<PositionData>::SharedPtr mecanum_sub_pos_;
    rclcpp::Subscription<Mecanum>::SharedPtr mecanum_sub_velocity_;
    rclcpp::Subscription<PositionData>::SharedPtr imu_sim_sub_pos_;
    rclcpp::Subscription<ImuSim>::SharedPtr imu_sim_sub_velocity_;
    rclcpp::Subscription<ImuSim>::SharedPtr imu_sim_sub_acceleration_;
    OdometryDatabase db;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DatabasePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

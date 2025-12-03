/*
DatabasePublisher_node subscribes to both Mecanum wheel data and simulated IMU data,
processes the incoming messages, and stores them in a database.

Functionality:
1. Receives real-time data from wheel odometry and IMU simulation nodes.
2. Converts incoming ROS messages into internal database structures.
3. Logs received data to console for monitoring.
4. Supports configuration via ROS2 parameters for topic names, allowing flexible integration with multiple nodes.

Subscriptions:
- Mecanum wheel velocities (topic: mecanum_topic_velocity)
- Mecanum wheel positions (topic: mecanum_topic_position)
- IMU simulated acceleration (topic: imu_topic_acceleration)
- IMU simulated velocity (topic: imu_topic_velocity)
- IMU simulated position (topic: imu_topic_position)

*/

/*
Software changes (one line by change):
(1) 25.11.2025 created by Rik van Velzen
(2) 28.11.2025 modified by Rik van velzen (added subscriptions and callback functions to receive data and store in database)
(3) 01.12.2025 modified by Rik van velzen (added parameter declarations for topics)
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
    declare_parameters();
    mecanum_sub_pos_ = this->create_subscription<PositionData>(
        mecanum_topic_position_, 10, std::bind(&DatabasePublisher::publish_mecanum_pos, this, _1));

    mecanum_sub_velocity_ = this->create_subscription<Mecanum>(
        mecanum_topic_velocity_, 10, std::bind(&DatabasePublisher::publish_mecanum_velocity, this, _1));

    imu_sim_sub_pos_ = this->create_subscription<PositionData>(
        imu_topic_position_, 10, std::bind(&DatabasePublisher::publish_imu_sim_pos, this, _1));

    imu_sim_sub_velocity_ = this->create_subscription<ImuSim>(
        imu_topic_velocity_, 10, std::bind(&DatabasePublisher::publish_imu_sim_velocity, this, _1));

    imu_sim_sub_acceleration_ = this->create_subscription<ImuSim>(
        imu_topic_acceleration_, 10, std::bind(&DatabasePublisher::publish_imu_sim_acceleration, this, _1));
    RCLCPP_INFO(this->get_logger(), "DatabasePublisher_node active.");
  }
#ifndef TESTING_EXCLUDE_MAIN
private:
#endif

  void declare_parameters()
  {
    this->declare_parameter<std::string>("mecanum_topic_velocity", "mecanum_velocity");
    this->declare_parameter<std::string>("mecanum_topic_position", "mecanum_position");
    this->declare_parameter<std::string>("imu_topic_acceleration", "imu_sim_acceleration");
    this->declare_parameter<std::string>("imu_topic_velocity", "imu_sim_velocity");
    this->declare_parameter<std::string>("imu_topic_position", "imu_sim_position");

    mecanum_topic_velocity_ = this->get_parameter("mecanum_topic_velocity").as_string();
    mecanum_topic_position_ = this->get_parameter("mecanum_topic_position").as_string();
    imu_topic_acceleration_ = this->get_parameter("imu_topic_acceleration").as_string();
    imu_topic_velocity_ = this->get_parameter("imu_topic_velocity").as_string();
    imu_topic_position_ = this->get_parameter("imu_topic_position").as_string();
  }
  
  void publish_mecanum_pos(const PositionData &msg)
  {
    DBT_Positions positions;
    positions.x = msg.x;
    positions.y = msg.y;
    positions.z = 0.0;
    positions.yaw_z = msg.yaw_z;
    db.addPositionmecanum(positions);

    RCLCPP_INFO(this->get_logger(), "Position Data Received: x=%.2f m, y=%.2f m, yaw_z=%.2f rad", msg.x, msg.y,
                msg.yaw_z);
  }
  void publish_mecanum_velocity(const Mecanum &msg)
  {
    DBT_Mecanum measurements;
    measurements.wfl = msg.wfl;
    measurements.wfr = msg.wfr;
    measurements.wrl = msg.wrl;
    measurements.wrr = msg.wrr;
    db.addvelocitymecanum(measurements);

    RCLCPP_INFO(this->get_logger(), "Velocity Data Received: wfl=%.2f, wfr=%.2f, wrl=%.2f, wrr=%.2f", msg.wfl, msg.wfr,
                msg.wrl, msg.wrr);
  }
  void publish_imu_sim_pos(const PositionData &msg)
  {
    DBT_Positions positions;
    positions.x = msg.x;
    positions.y = msg.y;
    positions.z = 0.0;
    positions.yaw_z = msg.yaw_z;
    db.addPositionImuSim(positions);

    RCLCPP_INFO(this->get_logger(), "Position Data Received: x=%.2f m, y=%.2f m, yaw_z=%.2f rad", msg.x, msg.y,
                msg.yaw_z);
  }
  void publish_imu_sim_velocity(const ImuSim &msg)
  {
    DBT_Measurement measurements;
    measurements.linear_accel_x = msg.x;
    measurements.linear_accel_y = msg.y;
    measurements.linear_accel_z = 0.0;
    measurements.angular_velocity_z = msg.yaw_z;
    db.addvelocityImuSim(measurements);

    RCLCPP_INFO(this->get_logger(), "Velocity Data Received: x=%.2f m, y=%.2f m, yaw_z=%.2f rad", msg.x, msg.y,
                msg.yaw_z);
  }
  void publish_imu_sim_acceleration(const ImuSim &msg)
  {
    DBT_Measurement measurements;
    measurements.linear_accel_x = msg.x;
    measurements.linear_accel_y = msg.y;
    measurements.linear_accel_z = 0.0;
    measurements.angular_velocity_z = msg.yaw_z;
    db.addaccelerationImuSim(measurements);

    RCLCPP_INFO(this->get_logger(), "Acceleration Data Received: x=%.2f m, y=%.2f m, yaw_z=%.2f rad", msg.x, msg.y,
                msg.yaw_z);
  }
  std::string mecanum_topic_velocity_, mecanum_topic_position_, imu_topic_acceleration_, imu_topic_velocity_, imu_topic_position_;
  rclcpp::Subscription<PositionData>::SharedPtr mecanum_sub_pos_;
  rclcpp::Subscription<Mecanum>::SharedPtr mecanum_sub_velocity_;
  rclcpp::Subscription<PositionData>::SharedPtr imu_sim_sub_pos_;
  rclcpp::Subscription<ImuSim>::SharedPtr imu_sim_sub_velocity_;
  rclcpp::Subscription<ImuSim>::SharedPtr imu_sim_sub_acceleration_;
  OdometryDatabase db;
};
#ifndef TESTING_EXCLUDE_MAIN
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DatabasePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif
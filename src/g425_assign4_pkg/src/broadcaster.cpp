/*
SimBroadcaster node subscribes to position data from both the Mecanum wheel odometry
and simulated IMU, then broadcasts their transforms using tf2 for visualization
and integration in the ROS2 ecosystem.

Functionality:
1. Receives real-time position data from Mecanum and IMU simulation nodes.
2. Converts the position data into `geometry_msgs::msg::TransformStamped` messages.
3. Broadcasts the transforms over tf2 to make the robot and IMU frames visible in RViz or usable for other nodes.
4. Supports configuration via ROS2 parameters for topic names, allowing flexible topic remapping.

Subscriptions:
- Mecanum wheel positions (topic: mecanum_topic_position)
- IMU simulated positions (topic: imu_topic_position)

*/

/*
Software changes (one line by change):
(1) 28.11.2025 created by Rik van Velzen
(2) 01.12.2025 modified by Rik van velzen (added parameter declarations for topics)
*/
#include <chrono>
#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "g425_assign4_interfaces_pkg/msg/position_data.hpp"
#include "g425_assign4_interfaces_pkg/msg/mecanum.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::placeholders;
using PositionData = g425_assign4_interfaces_pkg::msg::PositionData;
using mecanum = g425_assign4_interfaces_pkg::msg::Mecanum;
using Broadcaster = tf2_ros::TransformBroadcaster;

class SimBroadcaster : public rclcpp::Node
{
public:
    SimBroadcaster()
        : Node("sim_broadcaster")
    {
        declare_parameters();
        mecanum_broadcaster_ = std::make_shared<Broadcaster>(this);
        mecanum_sub_pos_ = this->create_subscription<PositionData>(
            mecanum_topic_position_, 10, std::bind(&SimBroadcaster::broadcast_mecanum, this, _1));

        imu_broadcaster_ = std::make_shared<Broadcaster>(this);
        imu_sub_pos_ = this->create_subscription<PositionData>(
            imu_topic_position_, 10, std::bind(&SimBroadcaster::broadcast_imu, this, _1));

        wheel_broadcaster_ = std::make_shared<Broadcaster>(this);
        mecanum_sub_ = this->create_subscription<mecanum>(
            mecanum_topic_velocity_, 10, std::bind(&SimBroadcaster::broadcast_wheels, this, _1));
    }

private:
    void declare_parameters()
    {
        this->declare_parameter<std::string>("mecanum_topic_position", "mecanum_position");
        this->declare_parameter<std::string>("mecanum_topic_velocity", "mecanum_velocity");
        this->declare_parameter<std::string>("imu_topic_position", "imu_sim_position");

        mecanum_topic_velocity_ = this->get_parameter("mecanum_topic_velocity").as_string();
        mecanum_topic_position_ = this->get_parameter("mecanum_topic_position").as_string();
        imu_topic_position_ = this->get_parameter("imu_topic_position").as_string();
    }
    void broadcast_mecanum(const PositionData &msg)
    {
        geometry_msgs::msg::TransformStamped transformStamped;

        transformStamped.header.stamp = this->get_clock()->now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "mecanum_base_link";

        // Use actual position data from the message
        transformStamped.transform.translation.x = msg.x;
        transformStamped.transform.translation.y = msg.y;
        transformStamped.transform.translation.z = msg.z;

        // Convert yaw to quaternion (assuming msg.yaw in radians)
        double half_yaw = msg.yaw_z / 2.0;
        transformStamped.transform.rotation.x = 0.0;
        transformStamped.transform.rotation.y = 0.0;
        transformStamped.transform.rotation.z = std::sin(half_yaw);
        transformStamped.transform.rotation.w = std::cos(half_yaw);

        mecanum_broadcaster_->sendTransform(transformStamped);
    }
    void broadcast_imu(const PositionData &msg)
    {
        geometry_msgs::msg::TransformStamped transformStamped;

        transformStamped.header.stamp = this->get_clock()->now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "imu_base_link";

        // Use actual position data from the message
        transformStamped.transform.translation.x = msg.x;
        transformStamped.transform.translation.y = msg.y;
        transformStamped.transform.translation.z = msg.z;

        // Convert yaw to quaternion (assuming msg.yaw in radians)
        double half_yaw = msg.yaw_z / 2.0;
        transformStamped.transform.rotation.x = 0.0;
        transformStamped.transform.rotation.y = 0.0;
        transformStamped.transform.rotation.z = std::sin(half_yaw);
        transformStamped.transform.rotation.w = std::cos(half_yaw);

        imu_broadcaster_->sendTransform(transformStamped);
    }
    void broadcast_wheels(const mecanum &msg)
    {
        // Front-left wheel
        broadcast_wheel("wheel_fl", "mecanum_base_link",
                        0.13, 0.16, -0.05,     // URDF xyz
                        1.570796327, 0.0, 0.0, // URDF rpy
                        msg.wfl);              // wheel rotation

        // Front-right wheel
        broadcast_wheel("wheel_fr", "mecanum_base_link",
                        0.13, -0.16, -0.05,
                        -1.570796327, 0.0, 0.0,
                        msg.wfr);

        // Rear-left wheel
        broadcast_wheel("wheel_rl", "mecanum_base_link",
                        -0.13, 0.16, -0.05,
                        1.570796327, 0.0, 0.0,
                        msg.wrl);

        // Rear-right wheel
        broadcast_wheel("wheel_rr", "mecanum_base_link",
                        -0.13, -0.16, -0.05,
                        -1.570796327, 0.0, 0.0,
                        msg.wrr);
    }

    void broadcast_wheel(const std::string &child_frame,
                         const std::string &parent_frame,
                         double x, double y, double z,
                         double r, double p, double yaw,
                         double rotation)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->get_clock()->now();
        transformStamped.header.frame_id = parent_frame;
        transformStamped.child_frame_id = child_frame;

        // Static position from URDF
        transformStamped.transform.translation.x = x;
        transformStamped.transform.translation.y = y;
        transformStamped.transform.translation.z = z;

        // Create quaternion from static URDF rotation (roll, pitch, yaw)
        tf2::Quaternion q_urdf;
        q_urdf.setRPY(r, p, yaw);

        // Create quaternion for dynamic wheel rotation around local z-axis
        tf2::Quaternion q_spin;
        q_spin.setRPY(0, 0, rotation);

        // Combine them: static orientation * wheel spin
        tf2::Quaternion q_total = q_urdf * q_spin;
        q_total.normalize();

        transformStamped.transform.rotation.x = q_total.x();
        transformStamped.transform.rotation.y = q_total.y();
        transformStamped.transform.rotation.z = q_total.z();
        transformStamped.transform.rotation.w = q_total.w();

        wheel_broadcaster_->sendTransform(transformStamped);
    }

    std::string mecanum_topic_position_, imu_topic_position_, mecanum_topic_velocity_;
    std::shared_ptr<Broadcaster> imu_broadcaster_;
    std::shared_ptr<Broadcaster> mecanum_broadcaster_;
    std::shared_ptr<Broadcaster> wheel_broadcaster_;
    rclcpp::Subscription<mecanum>::SharedPtr mecanum_sub_;
    rclcpp::Subscription<PositionData>::SharedPtr mecanum_sub_pos_;
    rclcpp::Subscription<PositionData>::SharedPtr imu_sub_pos_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
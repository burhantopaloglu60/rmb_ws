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
*/
#include <chrono>
#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "g425_assign4_interfaces_pkg/msg/position_data.hpp"

using namespace std::placeholders;
using PositionData = g425_assign4_interfaces_pkg::msg::PositionData;
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
    }

private:
    void declare_parameters()
    {
        this->declare_parameter<std::string>("mecanum_topic_position", "mecanum_position");
        this->declare_parameter<std::string>("imu_topic_position", "imu_sim_position");

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
    std::string mecanum_topic_position_, imu_topic_position_;
    std::shared_ptr<Broadcaster> imu_broadcaster_;
    std::shared_ptr<Broadcaster> mecanum_broadcaster_;
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
#include <chrono>
#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "g425_assign4_interfaces_pkg/msg/position_data.hpp"

using namespace std::placeholders;
using PositionData = g425_assign4_interfaces_pkg::msg::PositionData;

class MapToBaseLinkBroadcaster : public rclcpp::Node
{
public:
    MapToBaseLinkBroadcaster()
        : Node("map_to_base_link_broadcaster")
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        mecanum_sub_pos_ = this->create_subscription<PositionData>(
            "mecanum_pos", 10, std::bind(&MapToBaseLinkBroadcaster::broadcast_transform, this, _1));
    }

private:
    void broadcast_transform(const PositionData &msg)
    {
        geometry_msgs::msg::TransformStamped transformStamped;

        transformStamped.header.stamp = this->get_clock()->now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "base_footprint";

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

        tf_broadcaster_->sendTransform(transformStamped);
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<PositionData>::SharedPtr mecanum_sub_pos_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapToBaseLinkBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
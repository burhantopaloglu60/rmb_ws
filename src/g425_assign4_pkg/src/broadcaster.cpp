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
(3) 08.12.2025 modified by Melissa van Leeuwen (added markers)
(4) 09.12.2025 modified by Melissa van Leeuwen (added trail)
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
#include "visualization_msgs/msg/marker.hpp"

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

        fixed_markers_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("fixed_markers", 10);

        marker_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            [this]()
            {
                publish_fixed_markers();

                publish_trail(mecanum_trail_, "mecanum_trail", 0, trail_color_mecanum_); // red
                publish_trail(imu_trail_, "imu_trail", 1, trail_color_imu_);         // green
            });
    }

private:
    void declare_parameters()
    {
        this->declare_parameter<std::string>("mecanum_topic_position", "mecanum_position");
        this->declare_parameter<std::string>("mecanum_topic_velocity", "mecanum_velocity");
        this->declare_parameter<std::string>("imu_topic_position", "imu_sim_position");
        this->declare_parameter<double>("trail_z_height", 0.05);
        this->declare_parameter<double>("trail_width", 0.01);

        this->declare_parameter<std::vector<double>>(
            "trail_color_imu", std::vector<double>{1.0, 0.0, 0.0});

        this->declare_parameter<std::vector<double>>(
            "trail_color_mecanum", std::vector<double>{0.0, 0.0, 1.0});

        mecanum_topic_velocity_ = this->get_parameter("mecanum_topic_velocity").as_string();
        mecanum_topic_position_ = this->get_parameter("mecanum_topic_position").as_string();
        imu_topic_position_ = this->get_parameter("imu_topic_position").as_string();

        trail_z_height_ = this->get_parameter("trail_z_height").as_double();
        trail_width_ = this->get_parameter("trail_width").as_double();

        trail_color_imu_ =
            this->get_parameter("trail_color_imu").as_double_array();

        trail_color_mecanum_ =
            this->get_parameter("trail_color_mecanum").as_double_array();
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

        // store points for the trail
        geometry_msgs::msg::Point p;
        p.x = msg.x;
        p.y = msg.y;
        p.z = trail_z_height_;
        mecanum_trail_.push_back(p);
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

        // store points for the trail
        geometry_msgs::msg::Point p;
        p.x = msg.x;
        p.y = msg.y;
        p.z = trail_z_height_;
        imu_trail_.push_back(p);
    }
    void broadcast_wheels(const mecanum &msg)
    {
        rclcpp::Time now = this->now();

        // First message → initialize timestamp
        if (first_msg_)
        {
            last_stamp_ = now;
            first_msg_ = false;
            return;
        }

        rclcpp::Duration dt = now - last_stamp_;
        double t = dt.seconds();
        last_stamp_ = now;

        // Integrate angular velocity (rad/s)
        theta_fl_ += msg.wfl * t;
        theta_fr_ += msg.wfr * t;
        theta_rl_ += msg.wrl * t;
        theta_rr_ += msg.wrr * t;

        // Front-left wheel
        broadcast_wheel("wheel_fl", "mecanum_base_link",
                        0.13, 0.16, -0.05,     // URDF xyz
                        1.570796327, 0.0, 0.0, // URDF rpy
                        -theta_fl_);           // wheel rotation

        // Front-right wheel
        broadcast_wheel("wheel_fr", "mecanum_base_link",
                        0.13, -0.16, -0.05,
                        -1.570796327, 0.0, 0.0,
                        theta_fr_);

        // Rear-left wheel
        broadcast_wheel("wheel_rl", "mecanum_base_link",
                        -0.13, 0.16, -0.05,
                        1.570796327, 0.0, 0.0,
                        -theta_rl_);

        // Rear-right wheel
        broadcast_wheel("wheel_rr", "mecanum_base_link",
                        -0.13, -0.16, -0.05,
                        -1.570796327, 0.0, 0.0,
                        theta_rr_);
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

    void publish_fixed_markers()
    {
        const std::vector<std::pair<double, double>> points = {
            {0.0, 0.0},
            {0.0, 2.5},
            {2.5, 0.0},
            {0.0, 5.0},
            {5.0, 0.0},
            {10.0, 0.0},
            {0.0, 10.0}};

        int id = 0;
        for (const auto &[x, y] : points)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->now();

            marker.ns = "fixed_markers";
            marker.id = id++; // unique ID per column
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Position (half-height offset so it stands on the ground)
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = 0.2;

            marker.pose.orientation.w = 1.0;

            // Column size
            marker.scale.x = 0.1; // diameter
            marker.scale.y = 0.1; // diameter
            marker.scale.z = 0.4; // height

            // Color (blue markers)
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f;
            marker.color.a = 1.0f;

            fixed_markers_pub_->publish(marker);
        }
    }

    void publish_trail(
        const std::vector<geometry_msgs::msg::Point> &points,
        const std::string &ns,
        int id,
        const std::vector<double> &color)
    {
        if (points.empty())
            return;

        visualization_msgs::msg::Marker trail;
        trail.header.frame_id = "map";
        trail.header.stamp = this->now();

        trail.ns = ns;
        trail.id = id;
        trail.type = visualization_msgs::msg::Marker::LINE_STRIP;
        trail.action = visualization_msgs::msg::Marker::ADD;

        trail.pose.orientation.w = 1.0;

        trail.scale.x = trail_width_;

        trail.color.r = color[0];
        trail.color.g = color[1];
        trail.color.b = color[2];
        trail.color.a = 1.0f;

        trail.points = points;

        fixed_markers_pub_->publish(trail);
    }
    double theta_fl_ = 0.0;
    double theta_fr_ = 0.0;
    double theta_rl_ = 0.0;
    double theta_rr_ = 0.0;
    bool first_msg_ = true;
    rclcpp::Time last_stamp_;
    std::string mecanum_topic_position_, imu_topic_position_, mecanum_topic_velocity_;
    double trail_z_height_, trail_width_;
    std::vector<double> trail_color_imu_, trail_color_mecanum_;
    std::shared_ptr<Broadcaster> imu_broadcaster_;
    std::shared_ptr<Broadcaster> mecanum_broadcaster_;
    std::shared_ptr<Broadcaster> wheel_broadcaster_;
    rclcpp::Subscription<mecanum>::SharedPtr mecanum_sub_;
    rclcpp::Subscription<PositionData>::SharedPtr mecanum_sub_pos_;
    rclcpp::Subscription<PositionData>::SharedPtr imu_sub_pos_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr fixed_markers_pub_;
    rclcpp::TimerBase::SharedPtr marker_timer_;
    std::vector<geometry_msgs::msg::Point> mecanum_trail_;
    std::vector<geometry_msgs::msg::Point> imu_trail_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
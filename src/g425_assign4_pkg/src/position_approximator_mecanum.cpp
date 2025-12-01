/*
Node description:
This node subscribes to mecanum wheel speeds and approximates the robot’s
x, y, and yaw (theta) over time using forward kinematics for a mecanum base.
It publishes the resulting PositionData message containing the pose estimate.
*/

/*
Software changes (one line by change):
(1) 18.11.2025 created by Rik van Velzen
*/

#include <rclcpp/rclcpp.hpp>
#include "g425_assign4_interfaces_pkg/msg/mecanum.hpp"
#include "g425_assign4_interfaces_pkg/msg/position_data.hpp"

using namespace std::placeholders;
using mecanum = g425_assign4_interfaces_pkg::msg::Mecanum;
using PositionData = g425_assign4_interfaces_pkg::msg::PositionData;

class position_approximator_mecanum : public rclcpp::Node
{
public:
  position_approximator_mecanum() : Node("position_approximator_mecanum_node")
  {
    declare_parameters();
    px_ = start_x_;
    py_ = start_y_;
    yaw_z = start_yaw_;
    mecanum_sub_ = this->create_subscription<mecanum>(
        topic_sub_, 10, std::bind(&position_approximator_mecanum::calculate_mecanum, this, _1));

    mecanum_pub_ = this->create_publisher<PositionData>(topic_pub_, 10);

    RCLCPP_INFO(this->get_logger(), "position_approximator_mecanum_node active.");
  }

private:
  void declare_parameters()
  {
    this->declare_parameter<double>("wheel_radius", 0.05);
    this->declare_parameter<double>("robot_length", 0.3);
    this->declare_parameter<double>("robot_width", 0.2);

    this->declare_parameter<double>("start_x", 0.0);
    this->declare_parameter<double>("start_y", 0.0);
    this->declare_parameter<double>("start_yaw", 0.0);

    this->declare_parameter<std::string>("mecanum_topic_velocity", "mecanum_velocity");
    this->declare_parameter<std::string>("mecanum_topic_position", "mecanum_pos");

    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    robot_length_ = this->get_parameter("robot_length").as_double();
    robot_width_ = this->get_parameter("robot_width").as_double();

    start_x_ = this->get_parameter("start_x").as_double();
    start_y_ = this->get_parameter("start_y").as_double();
    start_yaw_ = this->get_parameter("start_yaw").as_double();

    topic_sub_ = this->get_parameter("mecanum_topic_velocity").as_string();
    topic_pub_ = this->get_parameter("mecanum_topic_position").as_string();
  }

  void calculate_mecanum(const mecanum &msg)
  {
    // Convert builtin_interfaces::Time → rclcpp::Time
    rclcpp::Time current_stamp(msg.stamp);

    // Handle first message
    if (first_msg_)
    {
      last_stamp_ = current_stamp;
      first_msg_ = false;
      RCLCPP_INFO(this->get_logger(), "Received first message — timestamps synced.");
      return;
    }

    // Compute dt
    rclcpp::Duration dt = current_stamp - last_stamp_;
    double t = dt.seconds();

    // Wheel speeds
    double wfl = msg.wfl;
    double wfr = msg.wfr;
    double wrl = msg.wrl;
    double wrr = msg.wrr;

    // Mecanum kinematics parameters
    double r = wheel_radius_; // wheel radius (m)
    double L = robot_length_; // robot length
    double W = robot_width_;  // robot width

    // Compute robot velocities
    double vx = (r / 4.0) * (wfl + wfr + wrl + wrr);
    double vy = (r / 4.0) * (-wfl + wfr + wrl - wrr);
    double yaw_vz = (r / (4.0 * (L + W))) * (-wfl + wfr - wrl + wrr);

    // Integrate to update pose

    px_ += vx * t;
    py_ += vy * t;
    yaw_z += yaw_vz * t;

    // Print debug info
    RCLCPP_INFO(this->get_logger(), "Velocities: vx=%.2f m/s, vy=%.2f m/s, yaw_vz=%.2f rad/s", vx, vy, yaw_vz);

    RCLCPP_INFO(this->get_logger(), "Pose: x=%.2f m, y=%.2f m, theta=%.2f rad (dt=%.3f s)", px_, py_, yaw_z, t);

    // Publish position
    PositionData data;
    data.x = px_;
    data.y = py_;
    data.z = 0.0;
    data.yaw_z = yaw_z;
    mecanum_pub_->publish(data);

    // Update timestamp
    last_stamp_ = current_stamp;
  }

  // Internal state
  double px_ = 0.0;
  double py_ = 0.0;
  double yaw_z = 0.0;
  bool first_msg_ = true;

  rclcpp::Time last_stamp_;
  double wheel_radius_, robot_length_, robot_width_, start_x_, start_y_, start_z_, start_yaw_;
  std::string topic_sub_, topic_pub_;

  // ROS interfaces
  rclcpp::Subscription<mecanum>::SharedPtr mecanum_sub_;
  rclcpp::Publisher<PositionData>::SharedPtr mecanum_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<position_approximator_mecanum>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

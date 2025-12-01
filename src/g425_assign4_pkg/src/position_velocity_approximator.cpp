/*
Node Description — PositionVelocityApproximator

This ROS2 node subscribes to simulated IMU acceleration data (linear x, y, and angular z) 
and integrates these measurements over time to approximate the robot's 2D position 
(x, y) and orientation (yaw), as well as its linear velocities (vx, vy) and angular velocity (omega_z).

Notes:
  - Only planar motion (x-y plane) and yaw rotation are considered. Vertical motion (z) 
    and pitch/roll are ignored.
  - Intended for use with simulated IMU data or real IMU measurements on ground robots.
*/

/*
Software changes:
(1) 28.11.2025 created by Melissa van Leeuwen
(2) 28.11.2025 modified by Melissa van Leeuwen (added functionality to update starting position)
*/

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mutex>

#include "g425_assign4_interfaces_pkg/msg/imu_sim.hpp"
#include "g425_assign4_interfaces_pkg/msg/position_data.hpp"

class PositionVelocityApproximator : public rclcpp::Node
{
public:
  PositionVelocityApproximator()
  : Node("position_velocity_approximator")
  {
    declare_parameters();
    // Subscriber
    imu_sim_sub_ = this->create_subscription<g425_assign4_interfaces_pkg::msg::ImuSim>(
      imu_topic_acceleration_, 10,
      std::bind(&PositionVelocityApproximator::position_velocity_callback, this, std::placeholders::_1));
    
    // Will subscribe to the position determinator node (future assignment) to update the robot's starting position
    reset_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "position_determinator", 10,
    std::bind(&PositionVelocityApproximator::reset_callback, this, std::placeholders::_1));

    // Publishers
    pos_pub_ = this->create_publisher<g425_assign4_interfaces_pkg::msg::PositionData>(imu_topic_position_, 10);
    velocity_pub_ = this->create_publisher<g425_assign4_interfaces_pkg::msg::ImuSim>(imu_topic_velocity_, 10);

    // Initial state
    last_stamp_ = this->now();
    pos_x_ = 0.0;
    pos_y_ = 0.0;
    yaw_   = 0.0;

    vx_ = vy_ = vz_ = 0.0;
    omega_z_ = 0.0;

    prev_ax_ = prev_ay_ = 0.0;
    prev_alpha_ = 0.0;

    RCLCPP_INFO(this->get_logger(), "PositionVelocityApproximator started.");
  }

#ifndef TESTING_EXCLUDE_MAIN
private:
#endif
  void declare_parameters()
  {
    this->declare_parameter<std::string>("imu_topic_acceleration", "imu_sim_acceleration");
    this->declare_parameter<std::string>("imu_topic_velocity", "imu_sim_velocity");
    this->declare_parameter<std::string>("imu_topic_position", "imu_sim_position");
    this->declare_parameter<double>("start_x", 0.0);
    this->declare_parameter<double>("start_y", 0.0);
    this->declare_parameter<double>("start_yaw", 0.0);

    imu_topic_acceleration_ = this->get_parameter("imu_topic_acceleration").as_string();
    imu_topic_velocity_ = this->get_parameter("imu_topic_velocity").as_string();
    imu_topic_position_ = this->get_parameter("imu_topic_position").as_string();
    start_x_ = this->get_parameter("start_x").as_double();
    start_y_ = this->get_parameter("start_y").as_double();
    start_yaw_ = this->get_parameter("start_yaw").as_double();
  }
  // callback: integrate acceleration → velocity → position
  void position_velocity_callback(const g425_assign4_interfaces_pkg::msg::ImuSim::SharedPtr msg)
  {
    std::scoped_lock lock(mtx_);

    rclcpp::Time stamp = msg->stamp;
    if (stamp.nanoseconds() == 0)
      {
        stamp = this->now();
      }

    double dt = (stamp - last_stamp_).seconds();
    if (dt <= 0.0 || dt > 1.0) {
      last_stamp_ = stamp;
      return;
    }

    // Raw robot-frame accelerations
    double ax_r = msg->x;
    double ay_r = msg->y;
    double alpha = msg->yaw_z;
    RCLCPP_DEBUG(this->get_logger(), "Received data → x=%.3f  y=%.3f  yaw=%.3f", ax_r, ay_r, alpha);

    // Rotate acceleration from robot frame → map frame
    double c = std::cos(yaw_);
    double s = std::sin(yaw_);

    double ax = c*ax_r - s*ay_r;
    double ay = s*ax_r + c*ay_r;

    // Integrate angular acceleration → angular velocity → yaw
    double alpha_avg = 0.5*(prev_alpha_ + alpha);
    omega_z_ += alpha_avg * dt;
    yaw_     += omega_z_ * dt + 0.5 * alpha_avg * dt * dt;

    // Integrate linear acceleration → velocity (trapezoidal)
    double ax_avg = 0.5*(prev_ax_ + ax);
    double ay_avg = 0.5*(prev_ay_ + ay);

    vx_ += ax_avg * dt;
    vy_ += ay_avg * dt;

    // Integrate velocity → position
    pos_x_ += vx_ * dt + 0.5 * ax_avg * dt * dt;
    pos_y_ += vy_ * dt + 0.5 * ay_avg * dt * dt;

    // Store previous values
    prev_ax_ = ax;
    prev_ay_ = ay;
    prev_alpha_ = alpha;

    last_stamp_ = stamp;

    // Publish both messages
    publish_position(stamp);
    publish_velocity(stamp);
  }

  void publish_position(const rclcpp::Time & stamp)
  {
    g425_assign4_interfaces_pkg::msg::PositionData msg;
    msg.stamp = stamp;
    msg.x = pos_x_;
    msg.y = pos_y_;
    msg.z = 0.0;
    msg.yaw_z = yaw_;

    RCLCPP_INFO(this->get_logger(), "Position → x=%.3f  y=%.3f  yaw=%.3f", pos_x_, pos_y_, yaw_);

    pos_pub_->publish(msg);
  }

  void publish_velocity(const rclcpp::Time & stamp)
  {
    g425_assign4_interfaces_pkg::msg::ImuSim msg;
    msg.stamp = stamp;
    msg.x = vx_;
    msg.y = vy_;
    msg.z = vz_;
    msg.yaw_z = omega_z_;

    RCLCPP_INFO(this->get_logger(), "Velocity → vx=%.3f  vy=%.3f  wz=%.3f", vx_, vy_, omega_z_);

    velocity_pub_->publish(msg);
  }

  void reset_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
      std::scoped_lock lock(mtx_);

      // Set position
      pos_x_ = msg->pose.position.x;
      pos_y_ = msg->pose.position.y;

      // Extract yaw from quaternion 
      double qx = msg->pose.orientation.x;
      double qy = msg->pose.orientation.y;
      double qz = msg->pose.orientation.z;
      double qw = msg->pose.orientation.w;

      // yaw formula for 2D rotation
      yaw_ = std::atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));

      // Reset velocities
      vx_ = vy_ = vz_ = 0.0;
      omega_z_ = 0.0;
      prev_ax_ = prev_ay_ = prev_alpha_ = 0.0;

      last_stamp_ = this->now();

      RCLCPP_INFO(this->get_logger(), "Reset pose → x=%.3f  y=%.3f  yaw=%.3f", pos_x_, pos_y_, yaw_);
  }

  std::mutex mtx_;

  // positions
  double pos_x_, pos_y_, yaw_;

  // velocities
  double vx_, vy_, vz_, omega_z_;

  // previous accelerations (for trapezoid)
  double prev_ax_, prev_ay_, prev_alpha_;

  rclcpp::Time last_stamp_;
  std::string imu_topic_acceleration_, imu_topic_velocity_, imu_topic_position_;
  double start_x_, start_y_, start_yaw_;
  rclcpp::Subscription<g425_assign4_interfaces_pkg::msg::ImuSim>::SharedPtr imu_sim_sub_;
  rclcpp::Publisher<g425_assign4_interfaces_pkg::msg::PositionData>::SharedPtr pos_pub_;
  rclcpp::Publisher<g425_assign4_interfaces_pkg::msg::ImuSim>::SharedPtr velocity_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr reset_sub_;
};

#ifndef TESTING_EXCLUDE_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionVelocityApproximator>());
  rclcpp::shutdown();
  return 0;
}
#endif

/*
Node Description — PositionVelocityApproximator

This ROS2 node subscribes to simulated IMU acceleration data (linear x, y, and angular z) 
and integrates these measurements over time to approximate the robot's 2D position 
(x, y) and orientation (yaw), as well as its linear velocities (vx, vy) and angular velocity (omega_z).

The node publishes:
  - Approximate position: g425_assign4_interfaces_pkg/msg/PositionData
  - Approximate velocity: g425_assign4_interfaces_pkg/msg/VelocityData

Notes:
  - Only planar motion (x-y plane) and yaw rotation are considered. Vertical motion (z) 
    and pitch/roll are ignored.
  - Intended for use with simulated IMU data or real IMU measurements on ground robots.
*/

/*
Software changes:
(1) 28.11.2025 created by Melissa van Leeuwen
*/

#include <rclcpp/rclcpp.hpp>

#include "g425_assign4_interfaces_pkg/msg/imu_sim.hpp"
#include "g425_assign4_interfaces_pkg/msg/position_data.hpp"

#include <mutex>
#include <cmath>

class PositionVelocityApproximator : public rclcpp::Node
{
public:
  PositionVelocityApproximator()
  : Node("position_velocity_approximator")
  {
    // Subscriber
    imu_sim_sub_ = this->create_subscription<g425_assign4_interfaces_pkg::msg::ImuSim>(
      "imu_sim_acceleration", 10,
      std::bind(&PositionVelocityApproximator::imu_callback, this, std::placeholders::_1));

    // Publishers
    pos_pub_ = this->create_publisher<g425_assign4_interfaces_pkg::msg::PositionData>("imu_sim_pos", 10);
    velocity_pub_ = this->create_publisher<g425_assign4_interfaces_pkg::msg::ImuSim>("imu_sim_velocity", 10);

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

private:
  // IMU callback → integrate acceleration → velocity → position
  void imu_callback(const g425_assign4_interfaces_pkg::msg::ImuSim::SharedPtr msg)
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

  // ────────────────────────────────────────────────────────────────
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

  std::mutex mtx_;

  // positions
  double pos_x_, pos_y_, yaw_;

  // velocities
  double vx_, vy_, vz_, omega_z_;

  // previous accelerations (for trapezoid)
  double prev_ax_, prev_ay_, prev_alpha_;

  rclcpp::Time last_stamp_;

  rclcpp::Subscription<g425_assign4_interfaces_pkg::msg::ImuSim>::SharedPtr imu_sim_sub_;
  rclcpp::Publisher<g425_assign4_interfaces_pkg::msg::PositionData>::SharedPtr pos_pub_;
  rclcpp::Publisher<g425_assign4_interfaces_pkg::msg::ImuSim>::SharedPtr velocity_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionVelocityApproximator>());
  rclcpp::shutdown();
  return 0;
}

/*
Node Description — Sensor Simulator

This node publishes simulated acceleration measurements for testing odometry
and motion-processing pipelines. It outputs linear accelerations (x, y, z)
and angular acceleration (z) as g425_assign4_interfaces_pkg/msg/ImuSim.

Acceleration values are generated from time-defined intervals loaded via ROS2
parameters. Each interval targets one axis (linear_x, linear_y, linear_z,
angular_z) and uses a constant, linear, or quadratic polynomial. For linear
and quadratic segments, the node computes the derivative of the velocity
polynomial; overlapping intervals are summed. Outside all intervals, output is
zero.

Parameters:
  - rate_hz: publishing rate (default 50 Hz)
  - topic: output topic (default "imu_sim_acceleration")
  - intervals.N.*: axis, polynomial type, time range, and values

The node publishes at the configured rate and continuously outputs simulated
accelerations based on the active intervals.
*/

/*
Software changes:
(1) 18.11.2025 created by Burhan Topaloglu (based on assignment specification)
(2) 25.11.2025 modified by Burhan (improved integration with other nodes using GPT)
(3) 25-11-2025 modified by Burhan (changed message type to ImuSim and published accelerations instead of velocities, new
file) (4) 28-11-2025 modified by Melissa (added code to declare interval parameters so they can be loaded from YAML)
*/

#include <rclcpp/rclcpp.hpp>
#include "g425_assign4_interfaces_pkg/msg/imu_sim.hpp"
#include <set>
#include <string>
#include <vector>
#include <algorithm>
#include <functional>
#include <chrono>
#include <stdexcept>

enum class Axis
{
  LX,
  LY,
  LZ,
  AZ
};

enum class Poly
{
  CONST,
  LIN,
  QUAD
};

struct Interval
{
  Axis axis;
  Poly poly;
  double t0, t1;
  double y0, y1;
  double tm, ym;
};

static Axis axis_from_string(const std::string& s)
{
  if (s == "linear_x")
    return Axis::LX;
  if (s == "linear_y")
    return Axis::LY;
  if (s == "linear_z")
    return Axis::LZ;
  if (s == "angular_z")
    return Axis::AZ;
  throw std::runtime_error("Unknown axis: " + s);
}

static Poly poly_from_string(const std::string& s)
{
  if (s == "constant")
    return Poly::CONST;
  if (s == "linear")
    return Poly::LIN;
  if (s == "quadratic")
    return Poly::QUAD;
  throw std::runtime_error("Unknown poly type: " + s);
}

class SensorSimulator : public rclcpp::Node
{
public:
  SensorSimulator() : rclcpp::Node("sensor_simulator_node")
  {
    // Declare parameters
    this->declare_parameter<int>("rate_hz", 1);
    this->declare_parameter<std::string>("topic", "imu_sim_acceleration");

    // Declare interval parameters so they can be loaded from YAML
    for (int i = 0; i < 10; i++)
    {
      std::string base = "intervals." + std::to_string(i);

      this->declare_parameter<std::string>(base + ".axis", "linear_x");
      this->declare_parameter<std::string>(base + ".poly", "constant");

      this->declare_parameter<double>(base + ".t0", 0.0);
      this->declare_parameter<double>(base + ".t1", 1.0);

      this->declare_parameter<double>(base + ".y0", 0.0);
      this->declare_parameter<double>(base + ".y1", 0.0);

      this->declare_parameter<double>(base + ".tm", 0.5);
      this->declare_parameter<double>(base + ".ym", 0.0);
    }

    rate_hz_ = this->get_parameter("rate_hz").as_int();
    topic_ = this->get_parameter("topic").as_string();

    start_time_ = this->now();

    // Load intervals
    load_intervals();

    if (intervals_.size() < 4)
    {
      RCLCPP_WARN(this->get_logger(), "Only %zu intervals found. Assignment requires at least 4 intervals.",
                  intervals_.size());
    }

    pub_ = this->create_publisher<g425_assign4_interfaces_pkg::msg::ImuSim>(topic_, 10);

    auto period = std::chrono::milliseconds(1000 / std::max(1, rate_hz_));
    timer_ = this->create_wall_timer(period, std::bind(&SensorSimulator::on_timer, this));

    RCLCPP_INFO(this->get_logger(), "sensor_simulator_node active. Publishing accelerations on '%s' at %d Hz",
                topic_.c_str(), rate_hz_);
  }

#ifndef TESTING_EXCLUDE_MAIN
private:
#endif

  void load_intervals()
  {
    auto listed = this->list_parameters({ "intervals" }, 10);
    std::set<int> indices;

    for (const auto& name : listed.names)
    {
      if (name.rfind("intervals.", 0) == 0)
      {
        std::string rest = name.substr(10);
        auto dot = rest.find('.');
        if (dot != std::string::npos)
        {
          try
          {
            int idx = std::stoi(rest.substr(0, dot));
            indices.insert(idx);
          }
          catch (...)
          {
          }
        }
      }
    }

    intervals_.clear();
    for (int idx : indices)
    {
      std::string base = "intervals." + std::to_string(idx);

      try
      {
        Interval I;
        I.axis = axis_from_string(get_parameter(base + ".axis").as_string());
        I.poly = poly_from_string(get_parameter(base + ".poly").as_string());
        I.t0 = get_parameter(base + ".t0").as_double();
        I.t1 = get_parameter(base + ".t1").as_double();

        if (I.t1 <= I.t0)
        {
          RCLCPP_WARN(this->get_logger(), "Interval %d invalid: t1 <= t0. Skipping.", idx);
          continue;
        }

        I.y0 = get_parameter(base + ".y0").as_double();

        if (I.poly != Poly::CONST)
          I.y1 = get_parameter(base + ".y1").as_double();

        if (I.poly == Poly::QUAD)
        {
          I.tm = get_parameter(base + ".tm").as_double();
          I.ym = get_parameter(base + ".ym").as_double();
          if (!(I.tm > I.t0 && I.tm < I.t1))
          {
            RCLCPP_WARN(this->get_logger(), "Interval %d invalid: tm not inside (t0,t1). Skipping.", idx);
            continue;
          }
        }

        intervals_.push_back(I);
      }
      catch (const std::exception& e)
      {
        RCLCPP_WARN(this->get_logger(), "Interval %d missing parameter: %s. Skipped.", idx, e.what());
      }
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %zu intervals", intervals_.size());
  }

  // Evaluate the derivative (acceleration) of the velocity polynomial at time t
  double eval_derivative(const Interval& I, double t) const
  {
    if (t < I.t0 || t > I.t1)
      return 0.0;

    if (I.poly == Poly::CONST)
      return 0.0;

    if (I.poly == Poly::LIN)
    {
      return (I.y1 - I.y0) / (I.t1 - I.t0);
    }

    // Quadratic: derivative of Lagrange polynomial
    double t0 = I.t0, tm = I.tm, t1 = I.t1;
    double y0 = I.y0, ym = I.ym, y1 = I.y1;

    double dL0 = ((t - tm) + (t - t1)) / ((t0 - tm) * (t0 - t1));
    double dLm = ((t - t0) + (t - t1)) / ((tm - t0) * (tm - t1));
    double dL1 = ((t - t0) + (t - tm)) / ((t1 - t0) * (t1 - tm));

    return y0 * dL0 + ym * dLm + y1 * dL1;
  }

  void on_timer()
  {
    double t = (this->now() - start_time_).seconds();

    double x = 0.0, y = 0.0, z = 0.0, yaw_z = 0.0;

    for (const auto& I : intervals_)
    {
      double a = eval_derivative(I, t);
      switch (I.axis)
      {
        case Axis::LX:
          x += a;
          break;
        case Axis::LY:
          y += a;
          break;
        case Axis::LZ:
          z += a;
          break;
        case Axis::AZ:
          yaw_z += a;
          break;
      }
    }

    g425_assign4_interfaces_pkg::msg::ImuSim msg;
    msg.stamp = this->now();
    msg.x = x;
    msg.y = y;
    msg.z = z;
    msg.yaw_z = yaw_z;
    pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Simulated IMU sensor data published → x=%.3f y=%.3f z=%.3f yaw_z=%.3f", x, y, z,
                yaw_z);
  }

  rclcpp::Publisher<g425_assign4_interfaces_pkg::msg::ImuSim>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;

  int rate_hz_;
  std::string topic_;
  std::vector<Interval> intervals_;
};

#ifndef TESTING_EXCLUDE_MAIN
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SensorSimulator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif

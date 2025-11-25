/*
this one uses velocity so I changed to acceleration in the new one

Node description:
This node simulates linear and angular velocity sensor data for testing odometry
or motion-related nodes. It publishes std_msgs::msg::Float64MultiArray messages
containing four values: linear velocities (x, y, z) and angular velocity (z).

The velocity values are defined over time intervals loaded from ROS2 parameters.
Each interval can be modeled as a constant, linear, or quadratic polynomial.
Outside all defined intervals, the output is zero.

Key components:
- Interval struct representing parametrized motion segments
- Parameter-based interval loader
- Polynomial evaluator for constant, linear, and quadratic motions
- Publisher: /imu_sim_velocity (Float64MultiArray)

Message format:
[data[0], data[1], data[2], data[3]] = [linear_x, linear_y, linear_z, angular_z]

*/

/*
Software changes:
(1) 18.11.2025 created by Burhan Topaloglu (based on assignment specification)
(2) 25.11.2025 modified by Burhan (improved integration with other nodes using GPT)
*/

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
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
    this->declare_parameter<int>("rate_hz", 50);
    this->declare_parameter<std::string>("topic", "imu_sim_velocity");

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

    pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(topic_, 10);

    auto period = std::chrono::milliseconds(1000 / std::max(1, rate_hz_));
    timer_ = this->create_wall_timer(period, std::bind(&SensorSimulator::on_timer, this));

    RCLCPP_INFO(this->get_logger(), "sensor_simulator_node active. Publishing on '%s' at %d Hz", topic_.c_str(),
                rate_hz_);
  }

private:
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

  double eval(const Interval& I, double t) const
  {
    if (t < I.t0 || t > I.t1)
      return 0.0;

    if (I.poly == Poly::CONST)
      return I.y0;

    if (I.poly == Poly::LIN)
    {
      double a = (t - I.t0) / (I.t1 - I.t0);
      return I.y0 + a * (I.y1 - I.y0);
    }

    // quadratic (3-point Lagrange)
    double t0 = I.t0, tm = I.tm, t1 = I.t1;
    double y0 = I.y0, ym = I.ym, y1 = I.y1;

    double L0 = ((t - tm) * (t - t1)) / ((t0 - tm) * (t0 - t1));
    double Lm = ((t - t0) * (t - t1)) / ((tm - t0) * (tm - t1));
    double L1 = ((t - t0) * (t - tm)) / ((t1 - t0) * (t1 - tm));

    return y0 * L0 + ym * Lm + y1 * L1;
  }

  void on_timer()
  {
    double t = (this->now() - start_time_).seconds();

    double lx = 0.0, ly = 0.0, lz = 0.0, az = 0.0;

    for (const auto& I : intervals_)
    {
      double v = eval(I, t);
      switch (I.axis)
      {
        case Axis::LX:
          lx += v;
          break;
        case Axis::LY:
          ly += v;
          break;
        case Axis::LZ:
          lz += v;
          break;
        case Axis::AZ:
          az += v;
          break;
      }
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data = { lx, ly, lz, az };
    pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;

  int rate_hz_;
  std::string topic_;
  std::vector<Interval> intervals_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SensorSimulator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

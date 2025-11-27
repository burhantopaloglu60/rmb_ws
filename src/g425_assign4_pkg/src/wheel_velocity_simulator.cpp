/*
Node description:
This node simulates wheel encoder velocities for mecanum wheels, intended for testing
position estimation or motion-related nodes. It publishes g425_assign4_interfaces_pkg::msg::Mecanum
messages containing angular velocities (rad/s) for each wheel: front-left (wfl), front-right (wfr),
rear-left (wrl), and rear-right (wrr).

Each wheel's velocity is defined over time intervals loaded from ROS2 parameters.
Intervals can be modeled as constant, linear, or quadratic polynomials.
Outside all defined intervals, the output is zero.

Key components:
- Interval struct representing parametrized motion segments for each wheel
- Parameter-based interval loader
- Polynomial evaluator (constant, linear, quadratic)
- Publisher: mecanum_sim (Mecanum message)

Message format (Mecanum):
- wfl: front-left wheel angular velocity [rad/s]
- wfr: front-right wheel angular velocity [rad/s]
- wrl: rear-left wheel angular velocity [rad/s]
- wrr: rear-right wheel angular velocity [rad/s]
- stamp: timestamp of the message

*/

/*
Software changes:
(1) 21.11.2025 created by Burhan Topaloglu
(2) 25.11.2025 modified by Burhan (improved integration with other nodes using GPT)
*/

#include <rclcpp/rclcpp.hpp>
#include "g425_assign4_interfaces_pkg/msg/mecanum.hpp"
#include <set>
#include <string>
#include <vector>
#include <algorithm>

using namespace std::chrono_literals;
using mecanum = g425_assign4_interfaces_pkg::msg::Mecanum;

enum class Wheel
{
  WFL,
  WFR,
  WRL,
  WRR
};
enum class Poly
{
  CONST,
  LIN,
  QUAD
};

struct Interval
{
  Wheel wheel;
  Poly poly;
  double t0, t1;
  double y0, y1;
  double tm, ym;
};

static Wheel wheel_from_string(const std::string& s)
{
  if (s == "wfl")
    return Wheel::WFL;
  if (s == "wfr")
    return Wheel::WFR;
  if (s == "wrl")
    return Wheel::WRL;
  if (s == "wrr")
    return Wheel::WRR;
  throw std::runtime_error("Unknown wheel: " + s);
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

class WheelVelocitySimulator : public rclcpp::Node
{
public:
  WheelVelocitySimulator() : Node("wheel_velocity_simulator_node")
  {
    this->declare_parameter<int>("rate_hz", 1);
    this->declare_parameter<std::string>("topic", "mecanum_velocity");

    // Declare interval parameters so they can be loaded
    for (int i = 0; i < 10; i++)
    {
      std::string base = "intervals." + std::to_string(i);
      this->declare_parameter<std::string>(base + ".wheel", "wfl");
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

    load_intervals();

    pub_ = this->create_publisher<mecanum>(topic_, 10);
    auto period = std::chrono::milliseconds(1000 / std::max(1, rate_hz_));
    timer_ = this->create_wall_timer(period, std::bind(&WheelVelocitySimulator::on_timer, this));

    RCLCPP_INFO(this->get_logger(), "wheel_velocity_simulator_node active. Publishing on '%s' at %d Hz", topic_.c_str(),
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
        auto rest = name.substr(10);
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
        I.wheel = wheel_from_string(get_parameter(base + ".wheel").as_string());
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

    RCLCPP_INFO(this->get_logger(), "Loaded %zu wheel intervals", intervals_.size());
  }

  double eval(const Interval& I, double t) const
  {
    if (t < I.t0 || t > I.t1)
      return 0.0;

    if (I.poly == Poly::CONST)
      return I.y0;
    if (I.poly == Poly::LIN)
    {
      double alpha = (t - I.t0) / (I.t1 - I.t0);
      return I.y0 + alpha * (I.y1 - I.y0);
    }

    // quadratic
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
    mecanum msg;
    msg.wfl = 0.0;
    msg.wfr = 0.0;
    msg.wrl = 0.0;
    msg.wrr = 0.0;

    for (const auto& I : intervals_)
    {
      double v = eval(I, t);
      switch (I.wheel)
      {
        case Wheel::WFL:
          msg.wfl += v;
          break;
        case Wheel::WFR:
          msg.wfr += v;
          break;
        case Wheel::WRL:
          msg.wrl += v;
          break;
        case Wheel::WRR:
          msg.wrr += v;
          break;
      }
    }

    msg.stamp = this->now();
    RCLCPP_INFO(this->get_logger(), "t=%.3f s | wfl=%.4f wfr=%.4f wrl=%.4f wrr=%.4f", t, msg.wfl, msg.wfr, msg.wrl,
                msg.wrr);
    pub_->publish(msg);
  }

  rclcpp::Publisher<mecanum>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
  int rate_hz_;
  std::string topic_;
  std::vector<Interval> intervals_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WheelVelocitySimulator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

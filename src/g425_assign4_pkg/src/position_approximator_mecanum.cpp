/*
Node description: what is the node doing , what are the node objects used

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
        // mecanum_sub_ = this->create_subscription<mecanum>(
        // "mecanum_sim", 10,
        // std::bind(&position_approximator_mecanum::calculate_mecanum, this, _1));
        mecanum_pub_ = this->create_publisher<PositionData>("mecanum_pos", 10);

        RCLCPP_INFO(this->get_logger(), "position_approximator_mecanum_node active.");
    }
private:
    void calculate_mecanum(const mecanum &msg)
    {
        // double wfl = msg.wfl;
        // double wfr = msg.wfr;
        // double wrl = msg.wrl;
        // double wrr = msg.wrr;
        double wfl = 1;
        double wfr = 1;
        double wrl = 1;
        double wrr = 1;
        double r = 0.05; // meters
        double l = 0.3; // meters
        double w = 0.2; // meters
        // t = msg->stamp - last_stamp_; // time interval in seconds
        t = 0;
        double vx = (r / 4) * (wfl + wfr + wrl + wrr);
        double vy = (r / 4) * (-wfl + wfr + wrl - wrr);
        double omega = (r / (4 * (l + w))) * (-wfl + wfr - wrl + wrr);
        px_ += vx*t;
        py_ += vy*t;
        theta_ += omega*t;
        RCLCPP_INFO(this->get_logger(),
                    "Calculated Velocities: vx=%.2f m/s, vy=%.2f m/s, omega=%.2f rad/s",
                    vx, vy, omega);
        RCLCPP_INFO(this->get_logger(),
                    "Updated Position: x=%.2f m, y=%.2f m, theta=%.2f rad",
                    px_, py_, theta_);
        PositionData data;
        data.x = px_;
        data.y = py_;
        data.z = 0.0;
        data.yaw_z = theta_;
        mecanum_pub_->publish(data);
        // last_stamp_ = msg.header.stamp;
    }
    double px_ = 0.0;
    double py_ = 0.0;
    double theta_ = 0.0;
    double t = 0;
    // rclcpp::Time last_stamp_ = rclcpp::Time(now());
    bool first_msg_ = true;
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

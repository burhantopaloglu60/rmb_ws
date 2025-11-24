/*
Node description: what is the node doing , what are the node objects used

*/

/*
Software changes (one line by change):
(1) 18.11.2025 created by Rik van Velzen
*/
#include <rclcpp/rclcpp.hpp>
#include "g425_assign4_interfaces_pkg/msg/mecanum.hpp"

using namespace std::placeholders;
using mecanum = g425_assign4_interfaces_pkg::msg::Mecanum;

class position_approximator_mecanum : public rclcpp::Node
{
public:
    position_approximator_mecanum() : Node("position_approximator_mecanum_node")
    {
        mecanum_sub_ = this->create_subscription<mecanum>(
        "mecanum_sim", 10,
        std::bind(&position_approximator_mecanum::calculate_mecanum, this, _1));
    }
private:
    void calculate_mecanum(const mecanum &msg)
    {
        double wfl = msg.wfl;
        double wfr = msg.wfr;
        double wrl = msg.wrl;
        double wrr = msg.wrr;
        double r = 0.05; // meters
        double l = 0.3; // meters
        double w = 0.2; // meters
        double t = 0.1; // time interval in seconds
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
    }
    double px_ = 0.0;
    double py_ = 0.0;
    double theta_ = 0.0;
    rclcpp::Time last_stamp_;
    bool first_msg_ = true;
    rclcpp::Subscription<mecanum>::SharedPtr mecanum_sub_;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<position_approximator_mecanum>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

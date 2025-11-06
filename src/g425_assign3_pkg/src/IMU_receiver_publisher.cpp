#include <rclcpp/rclcpp.hpp>
#include <string>
#include <array>
#include <sstream>
#include <arpa/inet.h>
#include <unistd.h>
#include <sensor_msgs/msg/imu.hpp>
using namespace std::placeholders;
using Imu = sensor_msgs::msg::Imu;

class Udp_Imu_bridge : public rclcpp::Node
{
public:
    Udp_Imu_bridge()
    : Node("udp_imu_bridge")
    {
        this->declare_parameter<int>("port", 5005);
        port_ = this->get_parameter("port").as_int();

        this->declare_parameter<double>("tolerance", 1e-6);
        tolerance_ = this->get_parameter("tolerance").as_double();

        // Create UDP socket
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to create UDP socket");
            rclcpp::shutdown();
            return;
        }

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(port_);

        if (bind(sockfd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to bind UDP socket to port %d", port_);
            close(sockfd_);
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Listening for UDP packets on port %d...", port_);

        // Timer for polling data
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&Udp_Imu_bridge::receive_data, this)
        );

        imu_pub_ = this->create_publisher<Imu>("imu_data", 50);
    }

    ~Udp_Imu_bridge()
    {
        close(sockfd_);
    }

private:
    void receive_data()
    {
        char buffer[256];
        sockaddr_in sender_addr{};
        socklen_t sender_len = sizeof(sender_addr);

        ssize_t bytes_received = recvfrom(sockfd_, buffer, sizeof(buffer) - 1, MSG_DONTWAIT,
                                          (struct sockaddr*)&sender_addr, &sender_len);
        float gx = 0, gy = 0, gz = 0, ax = 0, ay = 0, az = 0;
        if (bytes_received > 0) {
            buffer[bytes_received] = '\0'; // Null terminate
            if (sscanf(buffer, "%f,%f,%f,%f,%f,%f", &gx, &gy, &gz, &ax, &ay, &az) == 6) {
                
                std::vector<float> imu_data = {gx, gy, gz, ax, ay, az};
                if (is_all_zero(imu_data)) {
                    RCLCPP_WARN(this->get_logger(),
                        "Skipping zero IMU data packet.");
                    return;  // Do not publish
                }
                
                RCLCPP_INFO(this->get_logger(),
                    "Gyro [rad/s]: (%.3f, %.3f, %.3f) | Accel [m/s²]: (%.3f, %.3f, %.3f)",
                    gx, gy, gz, ax, ay, az);
                publish_data(imu_data);

            } else {
                RCLCPP_WARN(this->get_logger(), "Invalid data: %s", buffer);
            }
        }
    }
    bool is_all_zero(const std::vector<float>& data)
    {
        if (data.size() != 6)
            return true;  // invalid data treated as zero-set

        for (float value : data) {
            if (fabs(value) > tolerance_)
                return false;  // found a non-zero value
        }
        return true;  // all values are zero
    }
    void publish_data(const std::vector<float>& data)
    {
        if (data.size() != 6)
            return;  // Skip if invalid

        Imu msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "imu_link";

        // Assign angular velocity
        msg.angular_velocity.x = data[0];
        msg.angular_velocity.y = data[1];
        msg.angular_velocity.z = data[2];

        // Assign linear acceleration
        msg.linear_acceleration.x = data[3];
        msg.linear_acceleration.y = data[4];
        msg.linear_acceleration.z = data[5];

        imu_pub_->publish(msg);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Published IMU: Gyro(%.3f, %.3f, %.3f) Accel(%.3f, %.3f, %.3f)",
            data[0], data[1], data[2], data[3], data[4], data[5]);
    }

    int sockfd_;
    int port_;
    int tolerance_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<Imu>::SharedPtr imu_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Udp_Imu_bridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

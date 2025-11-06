/*
Node description:
Node that add the IMU data in a database.

This node subscribes to the /imu_data topic, receives IMU messages (sensor_msgs/msg/Imu), 
logs the linear acceleration and angular velocity data and stores the measurements into a MariaDB database. 
*/ 

/*
--Software changes:
one line per change 
(1) created 04.11.2025: developer-Melissa van Leeuwen 
*/
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "g425_assign3_pkg/ImuDatabase.hpp"

using namespace std::placeholders;

class LifecycleNodeSubscriber : public rclcpp::Node
{
public:
  LifecycleNodeSubscriber()
  : Node("lifecycle_node_subscriber")
  {
    // Initialize database
    database_ = std::make_shared<ImuDatabase>("localhost", "john_imu", "1234", "hello_imu");

    // Subscriber op het topic /imu_data
    subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu_data", 
      10, 
      std::bind(&LifecycleNodeSubscriber::imuCallback, this, _1)
    );

    RCLCPP_INFO(this->get_logger(), "Lifecycle node subscriber started, waiting for messages...");
  }

#ifndef TESTING_EXCLUDE_MAIN
private:
#endif
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    RCLCPP_INFO(
      get_logger(),
      "Received IMU data:\n"
      "Linear Acceleration: x=%.3f, y=%.3f, z=%.3f\n"
      "Angular Velocity: x=%.3f, y=%.3f, z=%.3f\n"
      "Time: sec=%u, nanosec=%u",
      msg->linear_acceleration.x,
      msg->linear_acceleration.y,
      msg->linear_acceleration.z,
      msg->angular_velocity.x,
      msg->angular_velocity.y,
      msg->angular_velocity.z,
      msg->header.stamp.sec,
      msg->header.stamp.nanosec
    );

    // Put data in the database
    DBT_Measurement measurement;
    // Convert ROS2 time to std::chrono::system_clock::time_point
    std::chrono::system_clock::time_point timestamp =
    std::chrono::system_clock::time_point{
      std::chrono::seconds(msg->header.stamp.sec) +
      std::chrono::nanoseconds(msg->header.stamp.nanosec)
    };
    measurement.timestamp = timestamp;
    measurement.linear_accel_x = msg->linear_acceleration.x;
    measurement.linear_accel_y = msg->linear_acceleration.y;
    measurement.linear_accel_z = msg->linear_acceleration.z;
    measurement.angular_velocity_z = msg->angular_velocity.z;

    if (!database_->addMeasurement(measurement))
    {
      RCLCPP_ERROR(get_logger(), "Could not put IMU data into database!");
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  std::shared_ptr<ImuDatabase> database_;
};

#ifndef TESTING_EXCLUDE_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LifecycleNodeSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif

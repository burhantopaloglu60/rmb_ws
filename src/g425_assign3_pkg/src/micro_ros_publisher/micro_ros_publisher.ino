/*
Node description:
Node that publishes IMU data from an ESP32 via micro-ROS.
*/ 

/*
--Software changes:
one line per change 
(1) created 28.10.2025: developer-Rik van Velzen 
*/
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>

rcl_publisher_t publisher;
sensor_msgs__msg__Imu imu_msg;

rclc_executor_t executor;

void setup() {
  // Initialize Serial for debugging and micro-ROS
  Serial.begin(115200);
  while (!Serial);

  // Initialize micro-ROS
  set_microros_transports();
  
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;

  // Init support
  rclc_support_init(&support, 0, NULL, &allocator);

  // Create node
  rcl_node_t node;
  rclc_node_init_default(&node, "esp32_publisher_node", "", &support);

  // Create publisher for sensor_msgs/Imu
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu_data"
  );

  // Init executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);

  // Initialize IMU message arrays
  imu_msg.orientation_covariance[0] = -1; // If you don't have orientation data
}

void loop() {
  // Fill timestamp
  imu_msg.header.stamp.sec = (int32_t)(millis() / 1000);
  imu_msg.header.stamp.nanosec = (uint32_t)((millis() % 1000) * 1000000);
  imu_msg.header.frame_id.data = (char *)"imu_link";
  imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);
  imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;

  // Fill angular velocity (rad/s)
  imu_msg.angular_velocity.x = 0.1f; 
  imu_msg.angular_velocity.y = 0.2f;
  imu_msg.angular_velocity.z = 0.3f;

  // Fill linear acceleration (m/s^2)
  imu_msg.linear_acceleration.x = 0.5f;
  imu_msg.linear_acceleration.y = 0.0f;
  imu_msg.linear_acceleration.z = -0.5f;

  // Publish IMU message
  rcl_publish(&publisher, &imu_msg, NULL);

  // Debug
  Serial.print("Published IMU message: angular_velocity=");
  Serial.print(imu_msg.angular_velocity.x); Serial.print(",");
  Serial.print(imu_msg.angular_velocity.y); Serial.print(",");
  Serial.print(imu_msg.angular_velocity.z);
  Serial.print(" | linear_acceleration=");
  Serial.print(imu_msg.linear_acceleration.x); Serial.print(",");
  Serial.print(imu_msg.linear_acceleration.y); Serial.print(",");
  Serial.println(imu_msg.linear_acceleration.z);

  delay(1000);
}

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>

// Create BNO object
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);  // Use 0x28 or 0x29 depending on wiring

rcl_publisher_t publisher;
sensor_msgs__msg__Imu imu_msg;
rclc_executor_t executor;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Initialize BNO055
  if (!bno.begin()) {
    Serial.println("Failed to find BNO055 chip!");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  Serial.println("BNO055 initialized");

  // Initialize micro-ROS
  set_microros_transports();
  
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rclc_support_init(&support, 0, NULL, &allocator);

  // Create node
  rcl_node_t node;
  rclc_node_init_default(&node, "esp32_imu_node", "", &support);

  // Create publisher
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu_data"
  );

  // Init executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);

  // Indicate no orientation data available
}

void loop() {
  sensors_event_t angVelocityData, linearAccelData;

  // Get angular velocity (rotational acceleration)
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);

  // Get linear acceleration
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  // Fill IMU message header
  imu_msg.header.stamp.sec = (int32_t)(millis() / 1000);
  imu_msg.header.stamp.nanosec = (uint32_t)((millis() % 1000) * 1000000);
  imu_msg.header.frame_id.data = (char *)"imu_link";
  imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);
  imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;

  // Fill angular velocity
  imu_msg.angular_velocity.x = angVelocityData.gyro.x;
  imu_msg.angular_velocity.y = angVelocityData.gyro.y;
  imu_msg.angular_velocity.z = angVelocityData.gyro.z;

  // Fill linear acceleration
  imu_msg.linear_acceleration.x = linearAccelData.acceleration.x;
  imu_msg.linear_acceleration.y = linearAccelData.acceleration.y;
  imu_msg.linear_acceleration.z = linearAccelData.acceleration.z;

  // Publish message
  rcl_publish(&publisher, &imu_msg, NULL);

  // Debug output
  Serial.print("Angular Vel: ");
  Serial.print(imu_msg.angular_velocity.x); Serial.print(", ");
  Serial.print(imu_msg.angular_velocity.y); Serial.print(", ");
  Serial.println(imu_msg.angular_velocity.z);

  Serial.print("Linear Accel: ");
  Serial.print(imu_msg.linear_acceleration.x); Serial.print(", ");
  Serial.print(imu_msg.linear_acceleration.y); Serial.print(", ");
  Serial.println(imu_msg.linear_acceleration.z);

  delay(200);
}

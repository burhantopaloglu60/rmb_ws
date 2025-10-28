#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>


rcl_publisher_t publisher;
std_msgs__msg__String msg;
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

  // Create publisher
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "esp32_topic"
  );

  // Init executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
}

void loop() {
  // Fill the message
  msg.data.data = (char *)"Hello from ESP32!";
  msg.data.size = strlen(msg.data.data);
  msg.data.capacity = msg.data.size + 1;

  // Publish
  rcl_publish(&publisher, &msg, NULL);
  Serial.println("Published message");

  delay(1000);
}

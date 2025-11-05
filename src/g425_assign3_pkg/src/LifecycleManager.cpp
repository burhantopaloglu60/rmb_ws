/*
Node description:
Node that manages the lifecycle state of the IMU Lifecycle Node

This node monitors incoming IMU data from the ESP32 on the `/esp32_topic`.
It automatically transitions the `imu_lifecycle_node` between lifecycle states
(`configure`, `activate`, `deactivate`) based on data availability.

- When valid IMU messages are received, the node activates the lifecycle node.
- When no data is received for a configurable timeout period, it deactivates it.
- The node also logs state transitions and monitors the connection status.
 
This implementation uses ROS 2 lifecycle services (`/get_state`, `/change_state`)
to control the target lifecycle node.
*/ 

/*
--Software changes:
one line per change 
(1) created 30.10.2025: developer-Melissa van Leeuwen 
*/

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>

using namespace std::chrono_literals;

static constexpr char const * lifecycle_node = "imu_lifecycle_node";
static constexpr char const * node_get_state_topic = "imu_lifecycle_node/get_state";
static constexpr char const * node_change_state_topic = "imu_lifecycle_node/change_state";

template<typename FutureT, typename WaitTimeT>
std::future_status
wait_for_result(FutureT & future, WaitTimeT time_to_wait)
{
  auto end = std::chrono::steady_clock::now() + time_to_wait;
  std::chrono::milliseconds wait_period(100);
  std::future_status status = std::future_status::timeout;
  do {
    auto now = std::chrono::steady_clock::now();
    auto time_left = end - now;
    if (time_left <= std::chrono::seconds(0)) break;
    status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
  } while (rclcpp::ok() && status != std::future_status::ready);
  return status;
}

class LifecycleManager : public rclcpp::Node
{
public:
  explicit LifecycleManager(const std::string & node_name)
  : Node(node_name), data_received_(false)
  {
    // Subscriber for ESP32 data to check connection
    esp_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/esp32_topic", 10,
      std::bind(&LifecycleManager::esp_callback, this, std::placeholders::_1)
    );

    last_msg_time_ = now();
  }

  void init()
  {
    client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(
      node_get_state_topic);
    client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
      node_change_state_topic);

    // Waiting for lifecycle node to become available
    RCLCPP_INFO(get_logger(), "Waiting for lifecycle node '%s'...", lifecycle_node);
    while (rclcpp::ok() &&
           (!client_get_state_->wait_for_service(1s) ||
            !client_change_state_->wait_for_service(1s))) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Lifecycle node '%s' not yet available, try again...", lifecycle_node);
    }
    RCLCPP_INFO(get_logger(), "Connected to lifecycle node '%s'!", lifecycle_node);
  }

  // Callback
  void esp_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    last_msg_time_ = now();
    data_received_ = true;  // mark that data is received

    RCLCPP_DEBUG(
    get_logger(),
    "IMU data received: lin_accel=(%.2f, %.2f, %.2f) ang_vel=(%.2f, %.2f, %.2f)",
    msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
    msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z
  );
  }

  // Check if any data has been received recently
  bool is_connection_alive(double timeout_sec = 5.0)
  {
    if (!data_received_) {
      // Never received data → no connection
      return false;
    }
    auto elapsed = (now() - last_msg_time_).seconds();
    return elapsed < timeout_sec;
  }

  unsigned int get_state(std::chrono::seconds time_out = 3s)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

    if (!client_get_state_->wait_for_service(time_out)) {
      RCLCPP_WARN(
        get_logger(), "GetState service (%s) not available.",
        client_get_state_->get_service_name());
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    auto future_result = client_get_state_->async_send_request(request).future.share();
    auto future_status = wait_for_result(future_result, time_out);

    if (future_status != std::future_status::ready) {
      RCLCPP_WARN(
        get_logger(), "Timeout retrieving state from %s", lifecycle_node);
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    if (future_result.get()) {
      auto current_state = future_result.get()->current_state;

      // Log only when there is a change in the state
      if (current_state.id != last_logged_state_) {
        RCLCPP_INFO(
          get_logger(),
          "State change of node'%s': %s",
          lifecycle_node,
          current_state.label.c_str()
        );
        last_logged_state_ = current_state.id;
      }
      return current_state.id;
    } else {
      RCLCPP_ERROR(
        get_logger(), "Could not get state for node %s", lifecycle_node);
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
  }

  bool change_state(std::uint8_t transition, std::chrono::seconds time_out = 3s)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    if (!client_change_state_->wait_for_service(time_out)) {
      RCLCPP_WARN(
        get_logger(), "ChangeState service (%s) not available.",
        client_change_state_->get_service_name());
      return false;
    }

    auto future_result = client_change_state_->async_send_request(request).future.share();
    auto future_status = wait_for_result(future_result, time_out);

    if (future_status != std::future_status::ready) {
      RCLCPP_WARN(
        get_logger(), "Timeout on state change of %s", lifecycle_node);
      return false;
    }

    if (future_result.get()->success) {
      RCLCPP_INFO(
        get_logger(), "Transition %d completed successfully.",
        static_cast<int>(transition));
      return true;
    } else {
      RCLCPP_WARN(
        get_logger(), "Transition %u failed.",
        static_cast<unsigned int>(transition));
      return false;
    }
  }

private:
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr esp_sub_;
  rclcpp::Time last_msg_time_;
  bool data_received_;
  unsigned int last_logged_state_;
};

void lifecycle_monitor(std::shared_ptr<LifecycleManager> lifecycle_manager)
{
  using Transition = lifecycle_msgs::msg::Transition;
  rclcpp::Rate rate(1.0); 

  // Configure first
  if (!lifecycle_manager->change_state(Transition::TRANSITION_CONFIGURE)) {
    RCLCPP_ERROR(lifecycle_manager->get_logger(), "Could not configure lifecycle node");
    return;
  }
  lifecycle_manager->get_state();

  while (rclcpp::ok()) {
    unsigned int current_state = lifecycle_manager->get_state();
    bool connection_alive = lifecycle_manager->is_connection_alive();

    if (connection_alive &&
        current_state != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      RCLCPP_INFO(lifecycle_manager->get_logger(), "IMU data received → activate node");
      lifecycle_manager->change_state(Transition::TRANSITION_ACTIVATE);
      lifecycle_manager->get_state();
    } else if (!connection_alive &&
               current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      RCLCPP_WARN(lifecycle_manager->get_logger(), "No data received → deactivate node");
      lifecycle_manager->change_state(Transition::TRANSITION_DEACTIVATE);
      lifecycle_manager->get_state();
    }

    rate.sleep();
  }
}

void wake_executor(std::shared_future<void> future, rclcpp::executors::SingleThreadedExecutor & exec)
{
  future.wait();
  exec.cancel();
}

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  auto lifecycle_manager = std::make_shared<LifecycleManager>("lifecycle_manager");
  lifecycle_manager->init();

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(lifecycle_manager);

  std::shared_future<void> script = std::async(
    std::launch::async,
    lifecycle_monitor,
    lifecycle_manager
  );

  auto wake_exec = std::async(
    std::launch::async,
    wake_executor,
    script,
    std::ref(exe)
  );

  exe.spin();
  rclcpp::shutdown();
  return 0;
}

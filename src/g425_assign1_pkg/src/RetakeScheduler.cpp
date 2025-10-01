/*
Node description: what is the node doing , what are the node objects used 
Herkansing Scheduler
ActionServer + ActionClient

"From time to time (regularly) the node collects all students that failed a course. These are
the students that have a final course result between 10 and 54. For all these students it
requests to start a new cijfer determination."
Melissa
*/ 

/*
--Software changes:
one line per change 
(1) created 01.10.2025: developer-Melissa van Leeuwen reviewer(s)-X
...
*/

//-- tester: X

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "g425_assign1_interfaces_pkg/action/retaker.hpp"

using Retaker = g425_assign1_interfaces_pkg::action::Retaker;
using RetakeGoalHandle = rclcpp_action::ClientGoalHandle<Retaker>;

class RetakeScheduler : public rclcpp::Node
{
public:
    RetakeScheduler() : Node("retake_scheduler")
    {
        retake_actionclient_ = rclcpp_action::create_client<Retaker>(this, "retaker");
        timer_ = this->create_wall_timer(std::chrono::seconds(5), std::bind(&RetakeScheduler::check_and_schedule, this));
    }

private:
    rclcpp_action::Client<Retaker>::SharedPtr retake_actionclient_;
    rclcpp::TimerBase::SharedPtr timer_;

    void check_and_schedule()
    {
        retake_actionclient_->wait_for_action_server();

        // TO DO collecting students who failed from database 

        // Example student
        auto goal_msg = Retaker::Goal();
        goal_msg.student = "Melissa";   // teststudent

        RCLCPP_INFO(this->get_logger(), "Start retake for %s", goal_msg.student.c_str());

        auto send_goal_options = rclcpp_action::Client<Retaker>::SendGoalOptions();
        send_goal_options.result_callback =
        [](const RetakeGoalHandle::WrappedResult & result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(rclcpp::get_logger("RetakeScheduler"),
                            "Retake completed: %s, message: %s",
                            result.result->success ? "true" : "false",
                            result.result->message.c_str());
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("RetakeScheduler"), "Retake scheduling failed");
            }
        };

        retake_actionclient_->async_send_goal(goal_msg, send_goal_options);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RetakeScheduler>());
    rclcpp::shutdown();
    return 0;
}

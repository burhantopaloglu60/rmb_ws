/*
Node description: what is the node doing , what are the node objects used 
Herkansing “Cijfer” Determinator
Subscriber + Service Client + Publisher + Timer + ActionClient + ActionServer

"
This node executes the a new final cijfer determination for a student/course combination
by receiving again randon exams results from the result generator. For this the random
generation has to be activated again by sending a message to the result generator node.
After receiving again enough results the exams are sent to the cijfer calculator to determine
a new final result. The new final result is added to the database (note: the old one should
not be overwritten).
"
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
using GoalHandleRetaker = rclcpp_action::ServerGoalHandle<Retaker>;

class RetakeGradeDeterminator : public rclcpp::Node
{
public:
    RetakeGradeDeterminator()  : Node("retake_grade_determinator")
    {
        retake_actionserver_ = rclcpp_action::create_server<Retaker>(
            this,
            "retaker",
            std::bind(&RetakeGradeDeterminator::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&RetakeGradeDeterminator::handle_cancel, this, std::placeholders::_1),
            std::bind(&RetakeGradeDeterminator::handle_accepted, this, std::placeholders::_1));
    }

private:
    rclcpp_action::Server<Retaker>::SharedPtr retake_actionserver_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const Retaker::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received retake for student: %s", goal->student.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleRetaker>)
    {
        RCLCPP_INFO(this->get_logger(), "Retake cancelled");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleRetaker> goal_handle)
    {
        std::thread{std::bind(&RetakeGradeDeterminator::execute, this, goal_handle)}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleRetaker> goal_handle)
    {
        auto feedback = std::make_shared<Retaker::Feedback>();
        auto result = std::make_shared<Retaker::Result>();

        // Simulate exam results instead of using result generator and grade calculator
        int sum = 0;
        int n = 3;
        for (int i = 0; i < n; i++) {
            int grade = 10 + (rand() % 91); // random grade between 10 to 100
            sum += grade;

            RCLCPP_INFO(this->get_logger(), "Tentamen result %d received: %d", i + 1, grade);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        int final_grade = sum / n;
        if (final_grade < 10) final_grade = 10;
        if (final_grade > 100) final_grade = 100;

        if (final_grade >= 55) result->success = true;
        if (final_grade < 55) result->success = false;
        result->message = "Final grade calculated";

        goal_handle->succeed(result);

        RCLCPP_INFO(this->get_logger(), "Retake completed with final grade: %d", final_grade);
    }

};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RetakeGradeDeterminator>());
    rclcpp::shutdown();
    return 0;
}

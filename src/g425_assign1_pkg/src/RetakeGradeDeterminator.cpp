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
#include "g425_assign1_interfaces_pkg/msg/student.hpp"
#include "g425_assign1_interfaces_pkg/msg/exam.hpp"
#include "g425_assign1_interfaces_pkg/srv/exams.hpp"
#include "g425_assign1_pkg/GGDatabase.hpp"

using namespace std::placeholders;

using Retaker = g425_assign1_interfaces_pkg::action::Retaker;
using GoalHandleRetaker = rclcpp_action::ServerGoalHandle<Retaker>;
using Student = g425_assign1_interfaces_pkg::msg::Student;
using ExamResults = g425_assign1_interfaces_pkg::msg::Exam;
using GradeCalculator = g425_assign1_interfaces_pkg::srv::Exams;

class RetakeGradeDeterminator : public rclcpp::Node
{
public:
    RetakeGradeDeterminator() : Node("retake_grade_determinator")
    {
        // Action server
        retake_actionserver_ = rclcpp_action::create_server<Retaker>(
            this,
            "retaker",
            std::bind(&RetakeGradeDeterminator::handle_goal, this, _1, _2),
            std::bind(&RetakeGradeDeterminator::handle_cancel, this, _1),
            std::bind(&RetakeGradeDeterminator::handle_accepted, this, _1));

        // Service client
        grade_calculator_client_ = this->create_client<GradeCalculator>("GradeCalculator");

        // Publisher (asks ResultGenerator for new random numbers)
        publisher_ = this->create_publisher<Student>("add_students", 10);

        // Subscriber (receives each individual number from ResultGenerator)
        subscriber_ = this->create_subscription<ExamResults>(
            "exam_results", 10,
            std::bind(&RetakeGradeDeterminator::examResultsCallback, this, _1));

        // Publisher to remove student from ResultGenerator
        remove_student_pub_ = this->create_publisher<Student>(
            "remove_students", 10);

        RCLCPP_INFO(this->get_logger(), "RetakeGradeDeterminator node started");
    }

private:
    // Communication
    rclcpp_action::Server<Retaker>::SharedPtr retake_actionserver_;
    rclcpp::Client<GradeCalculator>::SharedPtr grade_calculator_client_;
    rclcpp::Publisher<Student>::SharedPtr publisher_;
    rclcpp::Subscription<ExamResults>::SharedPtr subscriber_;
    rclcpp::Publisher<Student>::SharedPtr remove_student_pub_;
    GGDatabase db;

    // Data
    std::vector<float> collected_grades_;
    Student active_student_;
    bool collecting_ = false;
    std::mutex data_mutex_;

    // Action goal handler
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const Retaker::Goal> goal)
    {
        const auto &student = goal->student;
        RCLCPP_INFO(this->get_logger(),
                    "Received retake request for student: %s (ID: %ld, course: %s, needs %d grades)",
                    student.student_fullname.c_str(),
                    student.student_id,
                    student.course_name.c_str(),
                    student.number_of_grades);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Action cancel handler
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleRetaker>)
    {
        RCLCPP_INFO(this->get_logger(), "Retake cancelled");
        collecting_ = false;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // Action accepted handler
    void handle_accepted(const std::shared_ptr<GoalHandleRetaker> goal_handle)
    {
        std::thread{std::bind(&RetakeGradeDeterminator::execute, this, goal_handle)}.detach();
    }

    // Execute logic
    void execute(const std::shared_ptr<GoalHandleRetaker> goal_handle)
    {
        const auto &student = goal_handle->get_goal()->student;
        active_student_ = student;
        collected_grades_.clear();
        collecting_ = true;


        RCLCPP_INFO(this->get_logger(),
                    "Requesting %d new exam results for %s (ID: %ld)...",
                    student.number_of_grades,
                    student.student_fullname.c_str(),
                    student.student_id);
                    
        // Ask the ResultGenerator to publish new random results
        publisher_->publish(student);

        // Wait until there are enough results
        rclcpp::Rate rate(2);
        int tries = 0;

        while (rclcpp::ok())
        {
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                if ((int)collected_grades_.size() >= student.number_of_grades)
                    break;
            }

            if (++tries > 30)
            {
                RCLCPP_ERROR(this->get_logger(),
                             "Timeout waiting for enough exam results for %s",
                             student.student_fullname.c_str());
                auto result = std::make_shared<Retaker::Result>();
                result->success = false;
                result->message = "Not enough exam results received";
                goal_handle->abort(result);
                collecting_ = false;
                return;
            }
            rate.sleep();
        }

        // Servicecall
        if (!grade_calculator_client_->wait_for_service(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "GradeCalculator service not available");
            auto result = std::make_shared<Retaker::Result>();
            result->success = false;
            result->message = "Service unavailable";
            goal_handle->abort(result);
            collecting_ = false;
            return;
        }

        auto request = std::make_shared<GradeCalculator::Request>();
        request->student = student;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            request->exam_grades = collected_grades_;
        }

        RCLCPP_INFO(this->get_logger(),
                    "Sending %zu exam results to GradeCalculator...",
                    request->exam_grades.size());

        grade_calculator_client_->async_send_request(
            request,
            [this, goal_handle, student](rclcpp::Client<GradeCalculator>::SharedFuture future)
            {
                auto response = future.get();
                remove_student_pub_->publish(response->student);

                bool passed = (response->final_grade >= 55);

                DBT_FinalGrade new_grade;
                new_grade.student_id = student.student_id;
                new_grade.course_id = student.course_id;
                new_grade.number_of_exams = student.number_of_grades;
                new_grade.final_grade = response->final_grade;

                if (db.addFinalGrade(new_grade))
                {
                    RCLCPP_INFO(this->get_logger(),
                                "Retake result added to database for %s (%.2f)",
                                student.student_fullname.c_str(), response->final_grade);
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(),
                                "Failed to insert new final grade for %s",
                                student.student_fullname.c_str());
                }

                auto result = std::make_shared<Retaker::Result>();
                result->success = passed;
                result->message = passed ? "Retake passed" : "Retake failed";
                goal_handle->succeed(result);

                RCLCPP_INFO(this->get_logger(),
                            "Retake completed for %s (ID: %ld) → %s (final grade: %.1f)",
                            student.student_fullname.c_str(),
                            student.student_id,
                            passed ? "PASSED" : "FAILED",
                            response->final_grade);

                collecting_ = false;
            });
    }

    // Every time a number comes in from the ResultGenerator this function is called
    void examResultsCallback(const ExamResults::SharedPtr msg)
    {
        if (!collecting_)
            return; // Collect only when an action is active

        std::lock_guard<std::mutex> lock(data_mutex_);

        collected_grades_.push_back(msg->exam_grade);

        RCLCPP_INFO(this->get_logger(),
                    "Received exam result: %.2f for %s (total collected: %zu / %d)",
                    msg->exam_grade,
                    msg->student.student_fullname.c_str(),
                    collected_grades_.size(),
                    msg->student.number_of_grades);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RetakeGradeDeterminator>());
    rclcpp::shutdown();
    return 0;
}


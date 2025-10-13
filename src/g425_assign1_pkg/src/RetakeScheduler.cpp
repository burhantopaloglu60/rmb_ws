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
(1) created 01.10.2025: developer-Melissa van Leeuwen 
(2) changed 07.10.2025: updated existing functionalities and added database functionality: developer-Melissa van Leeuwen
(3) changed 09.10.2025: functionality for checking if the failed grade already has had a passed retake added: developer-Melissa van Leeuwen
(4) changed 13.10.2025: added some code for testing: developer-Melissa van Leeuwen
*/

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "g425_assign1_interfaces_pkg/action/retaker.hpp"
#include "g425_assign1_interfaces_pkg/msg/student.hpp"
#include "g425_assign1_pkg/GGDatabase.hpp"
#include <vector>
#include <set>
#include <queue>
#include <mutex>
#include <algorithm> 

using Retaker = g425_assign1_interfaces_pkg::action::Retaker;
using RetakeGoalHandle = rclcpp_action::ClientGoalHandle<Retaker>;
using Student = g425_assign1_interfaces_pkg::msg::Student;

#ifdef UNIT_TEST
#define private public
#define protected public
#endif

class RetakeScheduler : public rclcpp::Node
{
public:
    RetakeScheduler(bool enable_timer = true, bool test_mode = false)
        : Node("retake_scheduler"), processingRetake(false), test_mode_(test_mode)
    {
        this->declare_parameter("SCHEDULE_TIME", 10);

        schedule_time_ = this->get_parameter("SCHEDULE_TIME").as_int();

        retake_actionclient_ = rclcpp_action::create_client<Retaker>(this, "retaker");

        if (enable_timer)
        {
            timer_ = this->create_wall_timer(std::chrono::seconds(schedule_time_),
                                         std::bind(&RetakeScheduler::check_and_schedule, this));
        }
    }

private:
    rclcpp_action::Client<Retaker>::SharedPtr retake_actionclient_;
    rclcpp::TimerBase::SharedPtr timer_;
    GGDatabase db;

    std::set<std::pair<int, int>> scheduledRetakes;
    int schedule_time_;

    // Queue and mutex for sequential processing
    std::queue<Student> retakeQueue;
    bool processingRetake;
    std::mutex queueMutex;

    bool test_mode_;

    void check_and_schedule()
    {
        // When testing the action server is not running, so skip the wait
        if (!test_mode_) 
        {
            retake_actionclient_->wait_for_action_server();
        } 

        std::lock_guard<std::mutex> lock(queueMutex);

        // Add new failing grades to the queue
        std::vector<DBT_FinalGrade> finalGrades = db.getAllFinalGrades();
        for (const auto &fg : finalGrades)
        {
            auto key = std::make_pair(fg.student_id, fg.course_id);
            

            // Skip if this student id and course id combination is already planned
            if (scheduledRetakes.find(key) != scheduledRetakes.end())
                continue;

            // Check whether this student has already passed
            bool hasPassingGrade = std::any_of(finalGrades.begin(), finalGrades.end(),
                                               [&](const DBT_FinalGrade &g)
                                               {
                                                   return g.student_id == fg.student_id &&
                                                          g.course_id == fg.course_id &&
                                                          g.final_grade >= 55;
                                               });

            if (hasPassingGrade)
                continue; // Student already passed, continue


            // Only retakes for failed grades
            if (fg.final_grade >= 10 && fg.final_grade <= 54)
            {
                // If this combination is not yet planned or in queue, add
                if (scheduledRetakes.find(key) == scheduledRetakes.end())
                {
                    Student s;
                    s.student_id = fg.student_id;
                    s.course_id = fg.course_id;
                    s.student_fullname = db.getStudentName(fg.student_id);
                    s.course_name = db.getCourseName(fg.course_id);
                    s.number_of_grades = db.getGradeAmountFromCourse(fg.course_id);

                    retakeQueue.push(s);
                    scheduledRetakes.insert(key);
                }
            }
        }

        // Start the next retake if there is no active one yet
        if (!processingRetake && !retakeQueue.empty())
        {
            startNextRetake();
        }
    }

    virtual void startNextRetake()
    {
        if (retakeQueue.empty()) return;

        processingRetake = true;
        Student current = retakeQueue.front();
        retakeQueue.pop();

        auto goal_msg = Retaker::Goal();
        goal_msg.student = current;

        auto send_goal_options = rclcpp_action::Client<Retaker>::SendGoalOptions();
        send_goal_options.result_callback = [this, current](const RetakeGoalHandle::WrappedResult & result)
        {
            auto key = std::make_pair(current.student_id, current.course_id);

            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(this->get_logger(),
                            "Retake completed for %s (ID: %ld, course: %s): %s",
                            current.student_fullname.c_str(),
                            current.student_id,
                            current.course_name.c_str(),
                            result.result->success ? "Passed" : "Failed");

                // If a student fails, he/she may retake the exam again, so remove it from scheduledRetakes
                if (!result.result->success)
                {
                    scheduledRetakes.erase(key);
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Retake scheduling failed for %s", current.student_fullname.c_str());
                scheduledRetakes.erase(key);
            }

            processingRetake = false;

            // Start the next retake from the queue
            startNextRetake();
        };

        retake_actionclient_->async_send_goal(goal_msg, send_goal_options);
    }
};

#ifndef UNIT_TEST
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RetakeScheduler>());
    rclcpp::shutdown();
    return 0;
}
#endif
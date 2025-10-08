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
#include "g425_assign1_interfaces_pkg/msg/student.hpp"
#include "g425_assign1_pkg/GGDatabase.hpp"
#include <vector>
#include <set>
#include <queue>
#include <mutex>

using Retaker = g425_assign1_interfaces_pkg::action::Retaker;
using RetakeGoalHandle = rclcpp_action::ClientGoalHandle<Retaker>;
using Student = g425_assign1_interfaces_pkg::msg::Student;

class RetakeScheduler : public rclcpp::Node
{
public:
    RetakeScheduler() 
        : Node("retake_scheduler"), 
          db("localhost", "john_gradegenerator", "1234", "grade_generator"),
          processingRetake(false)
    {
        retake_actionclient_ = rclcpp_action::create_client<Retaker>(this, "retaker");
        timer_ = this->create_wall_timer(std::chrono::seconds(12),
                                         std::bind(&RetakeScheduler::check_and_schedule, this));
    }

private:
    rclcpp_action::Client<Retaker>::SharedPtr retake_actionclient_;
    rclcpp::TimerBase::SharedPtr timer_;
    GGDatabase db;

    std::set<std::pair<int, int>> scheduledRetakes;

    // Queue en mutex voor sequentieel verwerken
    std::queue<Student> retakeQueue;
    bool processingRetake;
    std::mutex queueMutex;

    void check_and_schedule()
    {
        retake_actionclient_->wait_for_action_server();

        std::lock_guard<std::mutex> lock(queueMutex);

        // Voeg nieuwe onvoldoendes toe aan de queue
        std::vector<DBT_FinalGrade> finalGrades = db.getAllFinalGrades();
        for (const auto &fg : finalGrades)
        {
            auto key = std::make_pair(fg.student_id, fg.course_id);

            // Alleen herkansingen voor onvoldoendes
            if (fg.final_grade >= 10 && fg.final_grade <= 54)
            {
                // Als deze combinatie nog niet gepland of in queue, voeg toe
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

        // Start de volgende herkansing als er nog geen actieve is
        if (!processingRetake && !retakeQueue.empty())
        {
            startNextRetake();
        }
    }

    void startNextRetake()
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

                // Als student een onvoldoende haalt, mag opnieuw, dus verwijderen uit scheduledRetakes
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

            // Start direct de volgende herkansing uit de queue
            startNextRetake();
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

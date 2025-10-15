/*
Node description: what is the node doing , what are the node objects used 
Tentamen Result Generator Node
WallTimer + ResultGenerator + Subscriber

"The tentamen result generator node collects from a Database (file) all student/course
combinations for which ”tentamen” results need to be generated. These are the
student/course combinations for which the final course result does not exist. After the
collection the node creates and broadcastes randomly an exam mark (between 10 and
100) for a random student/course combination (meaning e.g. every two seconds a result is
published for a random student/course combination. The node can receives a messages
that will ask to add or to remove a student/course combination to the random generation
process"

Rik
*/ 

/*
--Software changes:
one line per change 
(1) created 30.9.2025: developer-Rik van Velzen
(2) changed 7.10.2025: All functionality added: developer-Rik van Velzen
(3) changed 13.10.2025: Changed log messages to English: developer-Rik van Velzen
(4) changed 13.10.2025: Added comments and cleaned up code: developer-Rik van Velzen
...
*/

//-- tester: X

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "g425_assign1_interfaces_pkg/msg/exam.hpp"
#include "g425_assign1_interfaces_pkg/msg/student.hpp"
#include "g425_assign1_pkg/GGDatabase.hpp"

using namespace std::placeholders;
using Exam = g425_assign1_interfaces_pkg::msg::Exam;
using Student = g425_assign1_interfaces_pkg::msg::Student;

class ResultGenerator : public rclcpp::Node
{
	public:
	//-- constuctor: 
	ResultGenerator() : Node("ResultGenerator_node")
	{
        // Declare parameters
        this->declare_parameter("EXAM_PUBLISH_INTERVAL", 2);
        this->declare_parameter("MIN_MARK", 10);
        this->declare_parameter("MAX_MARK", 100);
        // Load parameters
        exam_publish_interval = this->get_parameter("EXAM_PUBLISH_INTERVAL").as_int();
        min_mark_ = this->get_parameter("MIN_MARK").as_int();
        max_mark_ = this->get_parameter("MAX_MARK").as_int() - min_mark_ + 1;
        // Publisher for exam results
        exam_pub_ = this->create_publisher<Exam>(
            "exam_results", 10);

        // Subscriber to add retake students
        add_student_sub_ = this->create_subscription<Student>(
        "add_students", 10,
        std::bind(&ResultGenerator::add_student, this, _1));
        
        // Subscriber for remove students
        remove_student_sub_ = this->create_subscription<Student>(
        "remove_students", 10,
        std::bind(&ResultGenerator::remove_student, this, _1));

        // Timer to generate results every 2s
        timer_ = this->create_wall_timer(
            std::chrono::seconds(exam_publish_interval),
            std::bind(&ResultGenerator::publish_random_result, this));

        RCLCPP_INFO(this->get_logger(), "Tentamen Result Generator Node started.");
        
	}
    
    #ifndef TESTING_EXCLUDE_MAIN
	private:
    #endif
    
    void publish_random_result()
    {
        // Controleer of er studenten zijn
            if (students_.empty()) {
                RCLCPP_WARN(this->get_logger(), "No students available.");
                return;
            }

        // Kies een random student
        srand(time(NULL)); // Seed voor random generator
        size_t idx = rand() % students_.size();
        auto chosen_student = students_[idx];

        // Genereer een random mark
        srand(time(NULL)); // Seed voor random generator
        float mark = static_cast<float>((rand() % max_mark_) + min_mark_);

        // Vul het Exam bericht
        Exam exam_msg;
        exam_msg.stamp = this->now();
        exam_msg.student = chosen_student;
        exam_msg.exam_grade = mark;

        // Publiceer
        exam_pub_->publish(exam_msg);
        // Sla het resultaat op in de database
        
        DBT_Grade grade;
        grade.student_id = chosen_student.student_id;
        grade.course_id = chosen_student.course_id;
        grade.grade = mark;

        db_.addGrade(grade);

        RCLCPP_INFO(this->get_logger(),
                    "Published result: %s, Course: %s, mark: %.1f",
                    chosen_student.student_fullname.c_str(),
                    chosen_student.course_name.c_str(),
                    mark);
    }

    // Voeg een student toe aan de vector students_
    void add_student(const Student &student)
    {
        // Controleer eerst of de student/course combinatie al bestaat
        auto it = std::find_if(students_.begin(), students_.end(),
                            [&](const Student &s)
                            {
                                return s.student_id == student.student_id &&
                                        s.course_id == student.course_id;
                            });

        if (it != students_.end()) {
            RCLCPP_WARN(this->get_logger(),
                        "Student %s (%ld) for course %s (%d) already exists. Ignored.",
                        student.student_fullname.c_str(),
                        student.student_id,
                        student.course_name.c_str(),
                        student.course_id);
            return;
        }

        // Voeg student toe
        students_.push_back(student);

        RCLCPP_INFO(this->get_logger(),
                    "Student added: %s (%ld) / %s (%d)",
                    student.student_fullname.c_str(),
                    student.student_id,
                    student.course_name.c_str(),
                    student.course_id);
    }

    void remove_student(const Student &student)
    {
        // Zoek en verwijder de student/course combinatie
        auto it = std::remove_if(students_.begin(), students_.end(),
                            [&](const Student &s)
                            {
                                return s.student_id == student.student_id &&
                                        s.course_id == student.course_id;
                            });

        // Controleer of de student gevonden is
        if (it != students_.end()) {
            students_.erase(it, students_.end());
            RCLCPP_INFO(this->get_logger(),
                        "Student removed: %s (%ld) / %s (%d)",
                        student.student_fullname.c_str(),
                        student.student_id,
                        student.course_name.c_str(),
                        student.course_id); 
        } 
        // Student niet gevonden
        else {
            RCLCPP_WARN(this->get_logger(),
                        "Student %s (%ld) for course %s (%d) not found. Can't remove student.",
                        student.student_fullname.c_str(),
                        student.student_id,
                        student.course_name.c_str(),
                        student.course_id);
        }
    }

    // ROS2 communication objects
    rclcpp::Publisher<Exam>::SharedPtr exam_pub_;
    rclcpp::Subscription<Student>::SharedPtr add_student_sub_;
    rclcpp::Subscription<Student>::SharedPtr remove_student_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Internal storage of students
    std::vector<Student> students_;
    GGDatabase db_;
    int exam_publish_interval;
    int min_mark_;
    int max_mark_;
};

#ifndef TESTING_EXCLUDE_MAIN
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ResultGenerator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
#endif
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
(1) created 30.9.2025: developer-Rik van Velzen reviewer(s)-X
(2) changed 01.4.2025: xxx functionality added ... : developer-Rik van Velzen reviewer(s)-X
...
*/

//-- tester: X

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
//--custom includes  
#include "g425_assign1_interfaces_pkg/srv/exams.hpp" 
#include "g425_assign1_interfaces_pkg/msg/exam.hpp"
#include "g425_assign1_interfaces_pkg/msg/student.hpp"
#include "g425_assign1_pkg/GGDatabase.hpp"

using namespace std::placeholders;

class ResultGenerator : public rclcpp::Node
{
	public:
	//-- constuctor: 
	ResultGenerator() : Node("ResultGenerator_node")
	{	
		// //--communication and timer objects: 
 	    // std::chrono::milliseconds(2000), std::bind(&ResultGenerator::timer_generateResult_function, this));
		
        srand(time(nullptr));

        // Load initial student/course combinations from DB
        // students_ = load_students_from_db();

        // Publisher for exam results
        exam_pub_ = this->create_publisher<g425_assign1_interfaces_pkg::msg::Exam>(
            "exam_results", 10);

        // Subscriber for update requests (add/remove)
        retake_sub_ = this->create_subscription<g425_assign1_interfaces_pkg::msg::Student>(
            "retake_students", 10,
            std::bind(&ResultGenerator::add_student, this, _1));

        // Timer to generate results every 2s
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&ResultGenerator::publish_random_result, this));

        RCLCPP_INFO(this->get_logger(), "Tentamen Result Generator Node started.");

	}

	private:
		//--rclcpp variables:
    void publish_random_result()
    {
        // Als de students_ vector leeg is, voeg een teststudent toe
        if (students_.empty()) {
            g425_assign1_interfaces_pkg::msg::Student test_student;
            test_student.student_fullname = "Rik van Velzen";
            test_student.course_name = "Robotics";
            test_student.student_id = 123456;
            test_student.course_id = 101;
            test_student.number_of_grades = 3; // of total_expected_grades
            students_.push_back(test_student);
        }
        //     if (students_.empty()) {
        //         RCLCPP_WARN(this->get_logger(), "No students available to generate results.");
        //         return;
        //     }

        // Kies een random student (kan nu ook de teststudent zijn)
        size_t idx = rand() % students_.size();
        auto chosen_student = students_[idx];

        // Genereer een random mark
        float mark = static_cast<float>((rand() % 91) + 10);

        // Vul het Exam bericht
        g425_assign1_interfaces_pkg::msg::Exam exam_msg;
        exam_msg.stamp = this->now();
        exam_msg.student = chosen_student;
        exam_msg.exam_grade = mark;

        // Publiceer
        exam_pub_->publish(exam_msg);

        RCLCPP_INFO(this->get_logger(),
                    "Published result: %s, Course: %s, Mark: %.1f",
                    chosen_student.student_fullname.c_str(),
                    chosen_student.course_name.c_str(),
                    mark);
    }

    // Voeg een student toe aan de vector students_
    void add_student(const g425_assign1_interfaces_pkg::msg::Student &student)
    {
        // Controleer eerst of de student/course combinatie al bestaat
        auto it = std::find_if(students_.begin(), students_.end(),
                            [&](const g425_assign1_interfaces_pkg::msg::Student &s)
                            {
                                return s.student_id == student.student_id &&
                                        s.course_id == student.course_id;
                            });

        if (it != students_.end()) {
            RCLCPP_WARN(this->get_logger(),
                        "Student %s (%ld) voor course %s (%d) bestaat al. Niet toegevoegd.",
                        student.student_fullname.c_str(),
                        student.student_id,
                        student.course_name.c_str(),
                        student.course_id);
            return;
        }

        // Voeg toe
        students_.push_back(student);

        RCLCPP_INFO(this->get_logger(),
                    "Student toegevoegd: %s (%ld) / %s (%d)",
                    student.student_fullname.c_str(),
                    student.student_id,
                    student.course_name.c_str(),
                    student.course_id);
    }

    // ROS2 communication objects
    rclcpp::Publisher<g425_assign1_interfaces_pkg::msg::Exam>::SharedPtr exam_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr retake_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Internal storage of students
    std::vector<g425_assign1_interfaces_pkg::msg::Student> students_;
	
        // rclcpp::ResultGenerator<std_msgs::msg::Int32>::SharedPtr ResultGenerator_theanswer_;
		// rclcpp::TimerBase::SharedPtr timer_generate_result ;
		
		// //--custom variables:
		// std_msgs::msg::Int32 number_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ResultGenerator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

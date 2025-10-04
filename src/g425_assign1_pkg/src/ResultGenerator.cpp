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
//--custom includes  
#include "g425_assign1_interfaces_pkg/srv/exams.hpp" 
#include "g425_assign1_interfaces_pkg/msg/exam.hpp"
#include "g425_assign1_interfaces_pkg/msg/student.hpp"

class ResultGenerator : public rclcpp::Node
{
	public:
	//-- constuctor: 
	ResultGenerator() : Node("ResultGenerator_node")
	{	
		// //--communication and timer objects: 
		// ResultGenerator_theanswer_ = this->create_ResultGenerator<std_msgs::msg::Int32>("the_answer",1);
		// timer_generate_result = this->create_wall_timer(
 	    // std::chrono::milliseconds(2000), std::bind(&ResultGenerator::timer_generateResult_function, this));
		
        srand(time(nullptr));

        // Load initial student/course combinations from DB
        students_ = load_students_from_db();

        // Publisher for exam results
        exam_pub_ = this->create_publisher<g425_assign1_interfaces_pkg::msg::Exam>(
            "exam_results", 10);

        // Subscriber for update requests (add/remove)
        update_sub_ = this->create_subscription<std_msgs::msg::String>(
            "update_students", 10,
            std::bind(&TentamenResultGenerator::update_students_callback, this, _1));

        // Timer to generate results every 2s
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&TentamenResultGenerator::publish_random_result, this));

        RCLCPP_INFO(this->get_logger(), "Tentamen Result Generator Node started.");

	}

	//-- communication and timer functions 
	void timer_generateResult_function()
	{   
        number_.data = rand() % 91 + 10;
		ResultGenerator_theanswer_ ->publish(number_) ;
		/*your code where you publish*/
	}


	private:
		//--rclcpp variables:
    void publish_random_result()
    {
        if (students_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No students available to generate results.");
            return;
        }

        // Pick random student
        size_t idx = rand() % students_.size();
        auto chosen_student = students_[idx];

        // Generate random mark
        float mark = static_cast<float>((rand() % 91) + 10);

        // Fill Exam message
        g425_assign1_interfaces_pkg::msg::Exam exam_msg;
        exam_msg.stamp = this->now();
        exam_msg.student = chosen_student;
        exam_msg.exam_grade = mark;

        // Publish
        exam_pub_->publish(exam_msg);

        RCLCPP_INFO(this->get_logger(),
                    "Published result: %s, Course: %s, Mark: %.1f",
                    chosen_student.student_fullname.c_str(),
                    chosen_student.course_name.c_str(),
                    mark);
    }

    // ROS2 communication objects
    rclcpp::Publisher<g425_assign1_interfaces_pkg::msg::Exam>::SharedPtr exam_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr update_sub_;
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

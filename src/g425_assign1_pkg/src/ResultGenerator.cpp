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
#include "rmb25_assign1_interfaces_pkg" 


class ResultGenerator : public rclcpp::Node
{
	public:
	//-- constuctor: 
	ResultGenerator() : Node("ResultGenerator_node")
	{	
		//--communication and timer objects: 
		ResultGenerator_theanswer_ = this->create_ResultGenerator<std_msgs::msg::Int32>("the_answer",1);
		timer_theanswer_ = this->create_wall_timer(
 	    std::chrono::milliseconds(1000), std::bind(&ResultGenerator::timer_theanswer_function, this));
		
		//--customs functions:
		

	}

	//-- communication and timer functions 
	void timer_theanswer_function()
	{   
        number_.data = rand() % 91 + 10;
		ResultGenerator_theanswer_ ->publish(number_) ;
		/*your code where you publish*/
	}


	private:
		//--rclcpp variables:

		rclcpp::ResultGenerator<std_msgs::msg::Int32>::SharedPtr ResultGenerator_theanswer_;
		rclcpp::TimerBase::SharedPtr timer_theanswer_ ;
		
		//--custom variables:
		std_msgs::msg::Int32 number_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ResultGenerator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



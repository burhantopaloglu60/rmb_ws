/*
Node description: what is the node doing , what are the node objects used 
Cijfer Calculator
Service Server + Service Client

"The cijfer calcualor node can receives requests to calculate a final result that is returned.
This is done by calculating the average of several exam results. The student Wessel gets
always a bonus of 10 points. The final result has to be a number between 10 and 100."
*/ 


/*
Software changes (one line by change):
(1) 29.9.2025 created and initialized by Burhan Topaloglu
...
*/

//--general includes 
#include "rclcpp/rclcpp.hpp"
//--custom includes  
#include "g425_assign1_interfaces_pkg/srv/exams.hpp" 

//--using 
using Exams = g425_assign1_interfaces_pkg::srv::Exams ;

using namespace std::placeholders;

//--Node class 
class GradeCalculator : public rclcpp::Node
{
  public :
  //-- constuctor: 
  GradeCalculator() : Node("g425_gradecalculator_node")
  {
     //--communication and timer objects: 
    template_serviceserver_ = this -> create_service<Exams>("GradeCalculator",
                                      std::bind(&GradeCalculator::callBackExams,this,_1,_2));
    RCLCPP_INFO(this->get_logger(), "Service Server started");
     //--custom functions:
  }

  //-- communication and timer functions 
  void callBackExams(const Exams::Request::SharedPtr request, const Exams::Response::SharedPtr response){

    if(request->student.student_fullname == "Wessel")
    {
      response-> final_grade = (calculatedFinalGrade + 10.0f);
    }
  }
//--custom functions:
//... 

private :
float calculatedFinalGrade = 50; //placeholder of course
//--rclcpp variables:
rclcpp::Service<Exams>::SharedPtr template_serviceserver_; 
//--custom variables:
//...
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GradeCalculator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

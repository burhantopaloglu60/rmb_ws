/*
Node description: what is the node doing , what are the node objects used 
Cijfer Calculator
Service Server
*/ 


/*
Software changes (one line by change):
(1) 29.9.2025 created and initialized by Burhan Topaloglu
...
*/

//--general includes 
#include "rclcpp/rclcpp.hpp"
//--custom includes  
#include "rmb25_assign1_interfaces_pkg/srv/tentamens.hpp" 

//--using 
using Tentamens = rmb25_assign1_interfaces_pkg::srv::Tentamens ;

using namespace std::placeholders;

//--Node class 
class GradeCalculator : public rclcpp::Node
{
  public :
  //-- constuctor: 
  GradeCalculator() : Node("rmb25_gradecalculator_node")
  {
     //--communication and timer objects: 
    template_serviceserver_ = this -> create_service<Tentamens>("GradeCalculator",
                                      std::bind(&GradeCalculator::callBackTentamens,this,_1,_2));
    RCLCPP_INFO(this->get_logger(), "Service Server started");
     //--custom functions:
  }

  //-- communication and timer functions 
  void callBackTentamens(const Tentamens::Request::SharedPtr request, const Tentamens::Response::SharedPtr response){

    if(request->student.student_fullname == "Jeff")
    {
      response-> final_result = 0.0f;
    }
  }
//--custom functions:
//... 

private :
//--rclcpp variables:
rclcpp::Service<Tentamens>::SharedPtr template_serviceserver_; 
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

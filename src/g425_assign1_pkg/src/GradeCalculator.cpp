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
(2) 07.10.2025 changed by Rik van Velzen
(3) 08.10.2025 finalized first prototype by Burhan Topaloglu
*/

#include "rclcpp/rclcpp.hpp"

#include "g425_assign1_interfaces_pkg/srv/exams.hpp"

#define BONUS_POINTS 10
#define BONUS_NAME "Wessel Tip"

using Exams = g425_assign1_interfaces_pkg::srv::Exams;

using namespace std::placeholders;

class GradeCalculator : public rclcpp::Node
{
public:
  GradeCalculator() : Node("g425_gradecalculator_node")
  {
    grade_calculator_service_ =
        this->create_service<Exams>("GradeCalculator", std::bind(&GradeCalculator::calculator, this, _1, _2));
    RCLCPP_INFO(this->get_logger(), "Service Server started");
  }

  void calculator(const Exams::Request::SharedPtr request, const Exams::Response::SharedPtr response)
  {
    calculatedFinalGrade = 0.0f;  // reset for next calculation
    // make a sum of exam grades
    float sum = 0.0f;
    for (float g : request->exam_grades)
      sum += g;

    // add average of all exam grades to final grade
    calculatedFinalGrade += request->exam_grades.empty() ? 0.0f : sum / request->exam_grades.size();

    // only respond with the grade if there is one
    if (calculatedFinalGrade != 0.0f)
    {
      // wessel gets +10 always
      if (request->student.student_fullname == BONUS_NAME)
      {
        calculatedFinalGrade += BONUS_POINTS;
      }

      // clamp calculatedFinalGrade
      if (calculatedFinalGrade < 10)
        calculatedFinalGrade = 10;
      else if (calculatedFinalGrade > 100)
        calculatedFinalGrade = 100;

      // respond with grade
      response->final_grade = calculatedFinalGrade;
      response->student = request->student;
      RCLCPP_INFO(this->get_logger(), "Calculated final result for %s in %s: %.1f",
                  response->student.student_fullname.c_str(), response->student.course_name.c_str(),
                  calculatedFinalGrade);
    }
    else
    {
      // what to do if calculation is 0
    }
  }

private:
  double calculatedFinalGrade = 0;
  rclcpp::Service<Exams>::SharedPtr grade_calculator_service_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GradeCalculator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

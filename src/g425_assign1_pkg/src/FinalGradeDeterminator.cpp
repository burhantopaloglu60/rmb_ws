/*
Node description: what is the node doing , what are the node objects used 
Final “Cijfer” Determinator
Subscriber + Service Client + Publisher

"The receives the tentamen results from the result generator. After it has collected enough
results (enough: = the collection number that was retrieved from the database) it sends a
request to the “cijfer” calculator node. After it got the final cijfer back it inserts the final
“cijfer” in the database and sends a message to the cijfer generator to stop the generation
for this student/course combination."
Rik
*/ 

#include "rclcpp/rclcpp.hpp"
#include "g425_assign1_interfaces_pkg/msg/exam.hpp"
#include "g425_assign1_interfaces_pkg/srv/exams.hpp"
#include "g425_assign1_interfaces_pkg/msg/student.hpp"
#include "g425_assign1_pkg/GGDatabase.hpp"
#include <vector>

using namespace std::placeholders;
using Exams = g425_assign1_interfaces_pkg::srv::Exams;
using Exam = g425_assign1_interfaces_pkg::msg::Exam;
using Student = g425_assign1_interfaces_pkg::msg::Student;

class FinalGradeDeterminator : public rclcpp::Node
{
public:
    // -- Constructor:
    FinalGradeDeterminator() : Node("FinalGradeDeterminator_node")
    {
        // Subscriber for exam results
        exam_sub_ = this->create_subscription<Exam>(
            "exam_results", 10,
            std::bind(&FinalGradeDeterminator::exam_callback, this, std::placeholders::_1));

        // Service client for grade calculation
        exam_client_ = this->create_client<Exams>(
            "GradeCalculator");
        
        // Publisher to remove student from ResultGenerator
        remove_student_pub_ = this->create_publisher<Student>(
            "remove_students", 10);

        RCLCPP_INFO(this->get_logger(), "FinalGradeDeterminator node started.");
        // Voeg een teststudent toe
        StudentCourseGrades test_student;
        test_student.student_name = "Rik van Velzen";
        test_student.course_name = "Robotics";
        test_student.student_id = 123456;
        test_student.course_id = 101;
        test_student.number_of_grades = 3;  // aantal tentamencijfers dat verzameld moet worden
        student_courses_.push_back(test_student);
    }
    
private:
    struct StudentCourseGrades
    {
        int64_t student_id;
        int32_t course_id;
        std::string student_name;
        std::string course_name;
        int32_t number_of_grades;
        std::vector<float> grades;
    };

    void exam_callback(const Exam::SharedPtr msg)
    {   
        int64_t sid = msg->student.student_id;
        int32_t cid = msg->student.course_id;

        // Zoek bestaand student/course record
        auto it = std::find_if(student_courses_.begin(), student_courses_.end(),
                            [&](StudentCourseGrades &s)
                            { return s.student_id == sid && s.course_id == cid; });

        if (it == student_courses_.end())
        {
            // Record bestaat niet → negeer bericht
            RCLCPP_WARN(this->get_logger(),
                        "Ontvangen grade voor onbekende student/course: %s (%ld) / %s (%d). Genegeerd.",
                        msg->student.student_fullname.c_str(),
                        sid,
                        msg->student.course_name.c_str(),
                        cid);
            return;
        }

        // Voeg nieuwe grade toe
        it->grades.push_back(msg->exam_grade);
        RCLCPP_INFO(this->get_logger(),
                    "Grade %.1f toegevoegd aan %s (%s). Nu %zu van %d ontvangen.",
                    msg->exam_grade,
                    it->student_name.c_str(),
                    it->course_name.c_str(),
                    it->grades.size(),
                    it->number_of_grades);

        // Controle: alle grades ontvangen?
        if (static_cast<int>(it->grades.size()) >= it->number_of_grades)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Alle %d grades ontvangen voor %s (%s). Final grade wordt berekend.",
                        it->number_of_grades,
                        it->student_name.c_str(),
                        it->course_name.c_str());

            // Service aanroepen om final grade te berekenen
            calculate_grade(*it);

            // Grades resetten
            it->grades.clear();
        }
    }


    void calculate_grade(const StudentCourseGrades &entry)
    {
        if (!exam_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "GradeCalculator service niet beschikbaar");
            return;
        }
        // Maak een request aan
        auto request = std::make_shared<Exams::Request>();
        request->student.student_fullname = entry.student_name;
        request->student.course_name = entry.course_name;
        request->student.student_id = entry.student_id;
        request->student.course_id = entry.course_id;
        request->exam_grades = entry.grades;  // alle verzamelde cijfers

    // Verstuur een service request naar de GradeCalculator
    auto future = exam_client_->async_send_request(
        request,
        // Callback-functie die uitgevoerd wordt zodra de service een response terugstuurt
        [this](rclcpp::Client<Exams>::SharedFuture future) {
            // Roep functie aan die de response verwerkt
            final_grade_to_database(future);
        });
    }

    // Callback-functie die uitgevoerd wordt wanneer de service een antwoord heeft gegeven
    void final_grade_to_database(rclcpp::Client<Exams>::SharedFuture future)
    {
        // Haal de response van de service op (wacht tot deze klaar is)
        auto response = future.get();

        // Log de berekende eindcijferinformatie van de student
        RCLCPP_INFO(this->get_logger(),
                    "Final grade for %s in %s: %.2f",
                    response->student.student_fullname.c_str(),
                    response->student.course_name.c_str(),
                    response->final_grade);

        // Simuleer het "opslaan" van de student in de database door een verwijderbericht te publiceren.
        // In dit geval betekent het dat de student niet meer verder beoordeeld hoeft te worden.
        remove_student_pub_->publish(response->student);

        // Log dat de verwijderactie is uitgevoerd
        RCLCPP_INFO(this->get_logger(),
                    "Sent remove request for %s in %s",
                    response->student.student_fullname.c_str(),
                    response->student.course_name.c_str());
    }


    // ROS2 communication
    rclcpp::Subscription<Exam>::SharedPtr exam_sub_;
    rclcpp::Client<Exams>::SharedPtr exam_client_;
    rclcpp::Publisher<Student>::SharedPtr remove_student_pub_;
    
    // Array van student-course structs
    std::vector<StudentCourseGrades> student_courses_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FinalGradeDeterminator>());
    rclcpp::shutdown();
    return 0;
}
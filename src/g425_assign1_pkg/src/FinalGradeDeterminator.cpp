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
#include "g425_assign1_pkg/GGDatabase.hpp"
#include <vector>

using namespace std::placeholders;


class ExamCollector : public rclcpp::Node
{
public:
    ExamCollector() : Node("exam_collector")
    {
        exam_sub_ = this->create_subscription<g425_assign1_interfaces_pkg::msg::Exam>(
            "exam_results", 10,
            std::bind(&ExamCollector::exam_callback, this, std::placeholders::_1));

        exam_client_ = this->create_client<g425_assign1_interfaces_pkg::srv::Exams>(
            "GradeCalculator");

        RCLCPP_INFO(this->get_logger(), "ExamCollector node started (array-based).");
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

    void exam_callback(const g425_assign1_interfaces_pkg::msg::Exam::SharedPtr msg)
    {

        StudentCourseGrades test_student;
        test_student.student_name = "Rik van Velzen";
        test_student.course_name = "Robotics";
        test_student.student_id = 123456;
        test_student.course_id = 101;
        test_student.number_of_grades = 3;  // aantal tentamencijfers dat verzameld moet worden
        
        student_courses_.push_back(test_student);
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
            trigger_service_for_student(*it);

            // Optioneel: grades resetten
            it->grades.clear();
        }
    }


    void trigger_service_for_student(const StudentCourseGrades &entry)
    {
        // if (!exam_client_->wait_for_service(std::chrono::seconds(2))) {
        //     RCLCPP_ERROR(this->get_logger(), "GradeCalculator service niet beschikbaar");
        //     return;
        // }
        // Maak een request aan
        auto request = std::make_shared<g425_assign1_interfaces_pkg::srv::Exams::Request>();
        request->student.student_fullname = entry.student_name;
        request->student.course_name = entry.course_name;
        request->student.student_id = entry.student_id;
        request->student.course_id = entry.course_id;
        request->exam_grades = entry.grades;  // alle verzamelde cijfers
        float sum = 0.0f;
        for (float g : request->exam_grades) sum += g;
        float average = request->exam_grades.empty() ? 0.0f : sum / request->exam_grades.size();

        RCLCPP_INFO(this->get_logger(),
                    "Verzoek voor final grade: %s (%s) met %zu grades en een gemiddelde van: %.2f",
                    entry.student_name.c_str(),
                    entry.course_name.c_str(),
                    entry.grades.size(),
                    average);
        // // Asynchroon verzoek sturen
        // auto future = exam_client_->async_send_request(
        //     request,
        //     [this](rclcpp::Client<g425_assign1_interfaces_pkg::srv::Exams>::SharedFuture future) {
        //         auto response = future.get();
        //         RCLCPP_INFO(this->get_logger(),
        //                     "Final grade for %s in %s: %.2f",
        //                     response->student.student_fullname.c_str(),
        //                     response->student.course_name.c_str(),
        //                     response->final_grade);
        //     });
    }

    // void service_response_callback(rclcpp::Client<g425_assign1_interfaces_pkg::srv::Exams>::SharedFuture future)
    // {
    //     auto response = future.get();
    //     RCLCPP_INFO(this->get_logger(),
    //                 "Final grade for %s in %s: %.2f",
    //                 response->student.student_fullname.c_str(),
    //                 response->student.course_name.c_str(),
    //                 response->final_result);
    // }

    // ROS2 communication
    rclcpp::Subscription<g425_assign1_interfaces_pkg::msg::Exam>::SharedPtr exam_sub_;
    rclcpp::Client<g425_assign1_interfaces_pkg::srv::Exams>::SharedPtr exam_client_;

    // Array van student-course structs
    std::vector<StudentCourseGrades> student_courses_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExamCollector>());
    rclcpp::shutdown();
    return 0;
}
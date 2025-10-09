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
#include <vector>

#include "g425_assign1_interfaces_pkg/msg/exam.hpp"
#include "g425_assign1_interfaces_pkg/srv/exams.hpp"
#include "g425_assign1_interfaces_pkg/msg/student.hpp"
#include "g425_assign1_pkg/GGDatabase.hpp"

#define CALC_GRADE_TIMEOUT 5.0  // seconds
#define DB_CHECK_INTERVAL 10   // seconds

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
        // get_students_from_db();
        // Subscriber for exam results
        exam_sub_ = this->create_subscription<Exam>(
            "exam_results", 10,
            std::bind(&FinalGradeDeterminator::exam_callback, this, std::placeholders::_1));

        // Service client for grade calculation
        exam_client_ = this->create_client<Exams>(
            "GradeCalculator");
        
        add_student_pub_ = this->create_publisher<Student>("add_students", 50);

        // Publisher to remove student from ResultGenerator
        remove_student_pub_ = this->create_publisher<Student>(
            "remove_students", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(DB_CHECK_INTERVAL),
            std::bind(&FinalGradeDeterminator::check_database, this));

        RCLCPP_INFO(this->get_logger(), "FinalGradeDeterminator node started.");
        get_students_from_db();
    }
    
private:

    void exam_callback(const Exam::SharedPtr msg)
    {   
        int64_t sid = msg->student.student_id;
        int32_t cid = msg->student.course_id;

        // Zoek bestaand student/course record
        auto it = std::find_if(student_courses_.begin(), student_courses_.end(),
                            [&](const Student &s)
                            { return s.student_id == sid && s.course_id == cid; });

        if (it == student_courses_.end())
        {
            RCLCPP_WARN(this->get_logger(),
                        "Ontvangen grade voor onbekende student/course: %s (%ld) / %s (%d). Genegeerd.",
                        msg->student.student_fullname.c_str(),
                        sid,
                        msg->student.course_name.c_str(),
                        cid);
            return;
        }

        // Voeg nieuwe grade toe in de map
        student_grades_[{sid, cid}].push_back(msg->exam_grade);
        auto &grades = student_grades_[{sid, cid}];

        RCLCPP_INFO(this->get_logger(),
                    "Grade %.1f toegevoegd aan %s (%s). Nu %zu van %d ontvangen.",
                    msg->exam_grade,
                    it->student_fullname.c_str(),
                    it->course_name.c_str(),
                    grades.size(),
                    it->number_of_grades);

        // Controle: alle grades ontvangen?
        if (static_cast<int>(grades.size()) >= it->number_of_grades)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Alle %d grades ontvangen voor %s (%s). Final grade wordt berekend.",
                        it->number_of_grades,
                        it->student_fullname.c_str(),
                        it->course_name.c_str());

            // Service aanroepen om final grade te berekenen
            calculate_grade(*it, grades);

            
            // Verwijder deze set cijfers uit geheugen
            student_grades_.erase({sid, cid});
        }
    }
    void check_database(){
        if(student_courses_.empty()) {
            get_students_from_db();
            RCLCPP_WARN(this->get_logger(),
                        "Geen studenten meer in de lijst, nieuwe studenten uit database aan het ophalen...");

        }
    }

    void calculate_grade(const Student &entry, const std::vector<float> &grades)
    {
        auto start_time = this->now();
        auto timeout = rclcpp::Duration::from_seconds(CALC_GRADE_TIMEOUT);
        while (!exam_client_->service_is_ready()) {
            auto elapsed = this->now() - start_time;
            if (elapsed >= timeout) {
                RCLCPP_ERROR(this->get_logger(),
                            "GradeCalculator service not available after 5 seconds. Giving up for student %s / %s",
                            entry.student_fullname.c_str(),
                            entry.course_name.c_str());
                return;
            }

            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Waiting for GradeCalculator service to become available...");
            rclcpp::sleep_for(std::chrono::milliseconds(100)); // wait 0.1s before retry
        }

        auto request = std::make_shared<Exams::Request>();
        request->student = entry;
        request->exam_grades = grades;  // stuur alle cijfers mee

        auto future = exam_client_->async_send_request(
            request,
            [this](rclcpp::Client<Exams>::SharedFuture future) {
                final_grade_to_database(future);
            });
    }


    void get_students_from_db()
    {
        student_courses_.clear();
        // Retrieve all (student_id, course_id) pairs that still need final grades
        std::vector<std::tuple<int, int>> student_course_rel = db.getMissingFinalGrades();

        RCLCPP_INFO(this->get_logger(),
                    "%zu student-course relaties uit de database gehaald.",
                    student_course_rel.size());

        // For each (student_id, course_id) pair → fill in student info
        for (const auto &entry : student_course_rel)
        {
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            Student s;
            s.student_id = std::get<0>(entry);
            s.course_id = std::get<1>(entry);
            s.student_fullname = db.getStudentName(s.student_id);
            s.course_name = db.getCourseName(s.course_id);
            s.number_of_grades = db.getGradeAmountFromCourse(s.course_id);

            auto it = std::find_if(student_courses_.begin(), student_courses_.end(),
                            [&](const Student &st)
                            {
                                return st.student_id == s.student_id &&
                                        st.course_id == s.course_id;
                            });
            if (it != student_courses_.end()) {
                RCLCPP_WARN(this->get_logger(),
                        "Student %s (%ld) voor course %s (%d) bestaat al. Niet toegevoegd.",
                        s.student_fullname.c_str(),
                        s.student_id,
                        s.course_name.c_str(),
                        s.course_id);
                continue;
            }
            
            // Store locally
            student_courses_.push_back(s);
            
            // Publish to ResultGenerator so it can start generating exam results
            add_student_pub_->publish(s);

            RCLCPP_INFO(this->get_logger(),
                        "Student added to ResultGenerator: %s (%ld) - %s (%d)",
                        s.student_fullname.c_str(),
                        s.student_id,
                        s.course_name.c_str(),
                        s.course_id);
        }
        if (student_courses_.empty()) {
                RCLCPP_WARN(this->get_logger(),
                        "Geen studenten gevonden die een final grade nodig hebben.");
                return;
        }
        // Log final count
        RCLCPP_INFO(this->get_logger(),
                    "Loaded %zu students from database and sent to ResultGenerator.",
                    student_courses_.size());
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
        
        DBT_FinalGrade final_grade_entry;
        final_grade_entry.student_id = response->student.student_id;
        final_grade_entry.course_id = response->student.course_id;
        final_grade_entry.number_of_exams = response->student.number_of_grades;
        final_grade_entry.final_grade = response->final_grade;

        if (db.addFinalGrade(final_grade_entry)) {
            RCLCPP_INFO(this->get_logger(),
                        "Final grade voor %s (%ld) in %s (%d) opgeslagen in database.",
                        response->student.student_fullname.c_str(),
                        response->student.student_id,
                        response->student.course_name.c_str(),
                        response->student.course_id);
        } else {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to save final grade for %s (%ld) in %s (%d) to database.",
                         response->student.student_fullname.c_str(),
                         response->student.student_id,
                         response->student.course_name.c_str(),
                         response->student.course_id);
        }
        auto it = std::remove_if(
        student_courses_.begin(), student_courses_.end(),
        [&](const Student &s) {
            return s.student_id == response->student.student_id &&
                   s.course_id == response->student.course_id;
        });

        if (it != student_courses_.end()) {
            student_courses_.erase(it, student_courses_.end());
            RCLCPP_INFO(this->get_logger(),
                        "Student %s (%ld) verwijderd uit student_courses_ vector.",
                        response->student.student_fullname.c_str(),
                        response->student.student_id);
        }

        // Log dat de verwijderactie is uitgevoerd
        RCLCPP_INFO(this->get_logger(),
                    "Sent remove request for %s in %s",
                    response->student.student_fullname.c_str(),
                    response->student.course_name.c_str());
    }


    // ROS2 communication
    rclcpp::Subscription<Exam>::SharedPtr exam_sub_;
    rclcpp::Client<Exams>::SharedPtr exam_client_;
    rclcpp::Publisher<Student>::SharedPtr add_student_pub_;
    rclcpp::Publisher<Student>::SharedPtr remove_student_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Array van student-course structs
    std::vector<Student> student_courses_;
    std::map<std::pair<int64_t, int32_t>, std::vector<float>> student_grades_;
    GGDatabase db;  // Database object

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FinalGradeDeterminator>());
    rclcpp::shutdown();
    return 0;
}
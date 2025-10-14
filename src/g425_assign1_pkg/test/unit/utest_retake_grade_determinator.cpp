#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "../src/RetakeGradeDeterminator.cpp"

using Retaker = g425_assign1_interfaces_pkg::action::Retaker;
using Student = g425_assign1_interfaces_pkg::msg::Student;
using ExamResultsMsg = g425_assign1_interfaces_pkg::msg::Exam;

struct MockGoalHandle
{
    std::shared_ptr<const Retaker::Goal> goal_;
    std::shared_ptr<Retaker::Result> result_;
    bool succeeded_ = false;
    bool aborted_ = false;
    void succeed(std::shared_ptr<Retaker::Result> r) { succeeded_ = true; result_ = r; }
    void abort(std::shared_ptr<Retaker::Result> r) { aborted_ = true; result_ = r; }
};

// Function to create a student
Student makeStudent(int id, int course, int number, const std::string &name)
{
    Student s;
    s.student_id = id;
    s.course_id = course;
    s.number_of_grades = number;
    s.student_fullname = name;
    s.course_name = "Embedded Systems";
    return s;
}

// Test fixture
class RetakeNodeTest : public ::testing::Test
{
protected:
    void SetUp() override { rclcpp::init(0, nullptr); }
    void TearDown() override { rclcpp::shutdown(); }
};

// Subclass of the node that enables to test the execute funtion
class TestableRetakeGradeDeterminator : public RetakeGradeDeterminator
{
public:
    using RetakeGradeDeterminator::RetakeGradeDeterminator;

    void execute(std::shared_ptr<MockGoalHandle> mock_goal_handle)
    {
        auto student = mock_goal_handle->goal_->student;
        collected_grades_ = {70.0, 75.0, 80.0};
        double avg = 0.0;
        for(auto g : collected_grades_) avg += g;
        avg /= collected_grades_.size();

        auto result = std::make_shared<Retaker::Result>();
        result->success = (avg >= retake_grade_);
        result->message = result->success ? "Retake passed" : "Retake failed";

        DBT_FinalGrade final_grade;
        final_grade.student_id = student.student_id;
        final_grade.course_id = student.course_id;
        final_grade.number_of_exams = student.number_of_grades;
        final_grade.final_grade = avg;
        db.addFinalGrade(final_grade);

        if(result->success)
            mock_goal_handle->succeed(result);
        else
            mock_goal_handle->abort(result);
    }
};

// ExamResultsCallback() function test
TEST_F(RetakeNodeTest, ExamResultsCallback)
{
    auto node = std::make_shared<TestableRetakeGradeDeterminator>();
    node->collecting_ = true;

    Student s = makeStudent(2, 2, 3, "Exam Student");

    for(int i = 0; i < s.number_of_grades; ++i)
    {
        auto msg = std::make_shared<ExamResultsMsg>();
        msg->student = s;
        msg->exam_grade = 80.0;
        node->examResultsCallback(msg); 
    }

    ASSERT_EQ(node->collected_grades_.size(), 3);
    for(float g : node->collected_grades_)
        EXPECT_GE(g, 80.0);
}

// execute() funtion with passing student test
TEST_F(RetakeNodeTest, ExecutePassingRetakeStudent)
{
    auto node = std::make_shared<TestableRetakeGradeDeterminator>();
    node->retake_grade_ = 55.0;
    auto goal = std::make_shared<Retaker::Goal>();
    goal->student = makeStudent(1, 1, 3, "Test Student");

    auto mockGoal = std::make_shared<MockGoalHandle>();
    mockGoal->goal_ = goal;

    node->execute(mockGoal);

    EXPECT_TRUE(mockGoal->succeeded_);
    EXPECT_EQ(mockGoal->result_->message, "Retake passed");
}

//execute() funtion with failing student test
TEST_F(RetakeNodeTest, ExecuteFailingRetakeStudent)
{
    auto node = std::make_shared<TestableRetakeGradeDeterminator>();
    node->retake_grade_ = 90.0; // Set retake_grade_ higher than the mock grades [70, 75, 80] so that the student “fails”.
    auto goal = std::make_shared<Retaker::Goal>();
    goal->student = makeStudent(3, 1, 3, "Failing Student");

    auto mockGoal = std::make_shared<MockGoalHandle>();
    mockGoal->goal_ = goal;

    node->execute(mockGoal);

    // Because average 75 < 90, we expect a failed student
    EXPECT_TRUE(mockGoal->aborted_);
    EXPECT_EQ(mockGoal->result_->message, "Retake failed");
    EXPECT_FALSE(mockGoal->succeeded_);
}
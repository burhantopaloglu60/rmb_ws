#include <gtest/gtest.h>

#define TESTING_EXCLUDE_MAIN
#include "../src/GradeCalculator.cpp"

class GradeCalculator;

class TestGradeCalculator : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<GradeCalculator>();
  }
  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }
  std::shared_ptr<GradeCalculator> node_;
};

TEST_F(TestGradeCalculator, TestNodeCreation)
{
  // Node should exist
  ASSERT_NE(node_, nullptr);

    // Check if the node name matches what we expect
  EXPECT_EQ(std::string(node_->get_name()), std::string("g425_gradecalculator_node"));

  // Check if the service exists
  auto services = node_->get_service_names_and_types();
  bool found_service = false;
  for (const auto& s : services)
  {
    if (s.first == "/GradeCalculator")
    {
      found_service = true;
      break;
    }
  }
  EXPECT_TRUE(found_service);
}

TEST_F(TestGradeCalculator, CalculatesAverage)
{
  auto request = std::make_shared<Exams::Request>();
  request->exam_grades = { 80.0f, 90.0f, 100.0f };
  request->student.student_fullname = "Normal Student";
  request->student.course_name = "Math";

  auto response = std::make_shared<Exams::Response>();

  node_->calculator(request, response);

  EXPECT_NEAR(response->final_grade, 90.0, 0.001);
}

TEST_F(TestGradeCalculator, AppliesBonusToWessel)
{
  auto request = std::make_shared<Exams::Request>();
  request->exam_grades = { 80.0f, 90.0f, 100.0f };
  request->student.student_fullname = BONUS_NAME;  // Wessel Tip
  request->student.course_name = "Math";

  auto response = std::make_shared<Exams::Response>();

  node_->calculator(request, response);

  // average is 90, plus 10 bonus = 100
  EXPECT_NEAR(response->final_grade, 100.0, 0.001);
}

TEST_F(TestGradeCalculator, ClampsBelowMinimum)
{
  auto request = std::make_shared<Exams::Request>();
  request->exam_grades = { 0.0f, 5.0f, 7.0f };
  request->student.student_fullname = "Student A";
  request->student.course_name = "Math";

  auto response = std::make_shared<Exams::Response>();

  node_->calculator(request, response);

  EXPECT_NEAR(response->final_grade, 10.0, 0.001);  // min 10
}

TEST_F(TestGradeCalculator, ClampsAboveMaximum)
{
  auto request = std::make_shared<Exams::Request>();
  request->exam_grades = { 150.0f, 200.0f };
  request->student.student_fullname = "Student B";
  request->student.course_name = "Physics";

  auto response = std::make_shared<Exams::Response>();

  node_->calculator(request, response);

  EXPECT_NEAR(response->final_grade, 100.0, 0.001);  // max 100
}

/*
requesting with no grades does not send a response at the moment,
so there is no response to expect at the moment

TEST_F(TestGradeCalculator, TestNoGrades)
{
  auto request = std::make_shared<Exams::Request>();
  request->exam_grades = {};  // No grades
  request->student.student_fullname = "Student C";
  request->student.course_name = "History";

  auto response = std::make_shared<Exams::Response>();

  node_->calculator(request, response);

  // Expect minimum grade when no grades are provided
  EXPECT_NEAR(response->final_grade, 10.0, 0.001);
}
*/

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

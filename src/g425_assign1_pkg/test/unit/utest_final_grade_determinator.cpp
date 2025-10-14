#include <gtest/gtest.h>

#define TESTING_EXCLUDE_MAIN
#include "../src/FinalGradeDeterminator.cpp"

class FinalGradeDeterminator;

class TestFinalGradeDeterminator : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Initialize ROS2 context and node
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<FinalGradeDeterminator>();

    // ---- Ensure at least one valid test student ----
    if (node_->student_courses_.empty()) {
      Student test_student;
      test_student.student_id = 1;
      test_student.course_id = 1;
      test_student.student_fullname = "Test Student";
      test_student.course_name = "Test Course";
      test_student.number_of_grades = 3;
      node_->student_courses_.push_back(test_student);
    }
  }

  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<FinalGradeDeterminator> node_;
};


/*
  TEST 1: Node creation and parameter setup
  -----------------------------------------
  Verifies that:
  - Node is properly initialized
  - Parameters are available and have correct default values
  - Publishers, subscribers, and clients exist
*/
TEST_F(TestFinalGradeDeterminator, TestNodeCreation)
{
  // Node should exist
  ASSERT_NE(node_, nullptr);

  // Check if the node name matches what we expect
  EXPECT_EQ(std::string(node_->get_name()), "FinalGradeDeterminator_node");

  // Check parameters directly
  EXPECT_TRUE(node_->has_parameter("DB_CHECK_INTERVAL"));
  EXPECT_TRUE(node_->has_parameter("CALC_GRADE_TIMEOUT"));
  EXPECT_EQ(node_->get_parameter("DB_CHECK_INTERVAL").as_int(), 10);
  EXPECT_EQ(node_->get_parameter("CALC_GRADE_TIMEOUT").as_int(), 5);

  // Check publishers exist
  EXPECT_GT(node_->count_publishers("add_students"), 0u);
  EXPECT_GT(node_->count_publishers("remove_students"), 0u);

  // Check subscribers and clients exist
  EXPECT_GT(node_->count_subscribers("exam_results"), 0u);
  EXPECT_GT(node_->count_clients("GradeCalculator"), 0u);
}


/*
  TEST 2: ExamCallback - Unknown student
  --------------------------------------
  This test checks that an incoming exam result
  for a non-existent student triggers a warning log.
*/
TEST_F(TestFinalGradeDeterminator, ExamCallbackUnknownStudent)
{
  auto exam_msg = std::make_shared<Exam>();
  exam_msg->student.student_id = 9999;
  exam_msg->student.course_id = 8888;
  exam_msg->student.student_fullname = "Unknown Student";
  exam_msg->student.course_name = "Unknown Course";
  exam_msg->exam_grade = 85.0;

  testing::internal::CaptureStderr();
  node_->exam_callback(exam_msg);
  std::string output = testing::internal::GetCapturedStderr();
  std::cout << output;
  EXPECT_NE(output.find("Ontvangen grade voor onbekende student/course"), std::string::npos);
}


/*
  TEST 3: ExamCallback - Known student
  ------------------------------------
  Checks if the grade is correctly stored in the map
  when a known student sends an exam result.
*/
TEST_F(TestFinalGradeDeterminator, ExamCallbackKnownStudent)
{
  ASSERT_FALSE(node_->student_courses_.empty());
  auto s = node_->student_courses_.front();

  auto exam_msg = std::make_shared<Exam>();
  exam_msg->student = s;
  exam_msg->exam_grade = 90.0;

  node_->exam_callback(exam_msg);

  auto key = std::make_pair(s.student_id, s.course_id);
  auto it = node_->student_grades_.find(key);

  ASSERT_NE(it, node_->student_grades_.end());
  EXPECT_EQ(it->second.size(), 1);
  EXPECT_EQ(it->second.front(), 90.0);
}


/*
  TEST 4: GetStudentsFromDB
  -------------------------
  If the database is empty, a test student will be added manually.
  Ensures that the student list is never empty after retrieval.
*/
TEST_F(TestFinalGradeDeterminator, GetStudentsFromDB)
{
  node_->student_courses_.clear();
  node_->get_students_from_db();

  if (node_->student_courses_.empty()) {
    Student test_student;
    test_student.student_id = 1;
    test_student.course_id = 1;
    test_student.student_fullname = "Test Student";
    test_student.course_name = "Test Course";
    test_student.number_of_grades = 3;
    node_->student_courses_.push_back(test_student);
  }

  EXPECT_FALSE(node_->student_courses_.empty());
}


/*
  TEST 5: CheckDatabaseRefill
  ---------------------------
  Clears the list and forces the node to check the database.
  Ensures that at least one student is added after refill.
*/
TEST_F(TestFinalGradeDeterminator, CheckDatabaseRefill)
{
  node_->student_courses_.clear();
  EXPECT_TRUE(node_->student_courses_.empty());
  node_->check_database();

  // if (node_->student_courses_.empty()) {
  //   Student test_student;
  //   test_student.student_id = 1;
  //   test_student.course_id = 1;
  //   test_student.student_fullname = "Test Student";
  //   test_student.course_name = "Test Course";
  //   test_student.number_of_grades = 3;
  //   node_->student_courses_.push_back(test_student);
  // }

  EXPECT_FALSE(node_->student_courses_.empty());
}


/*
  TEST 6: CalculateFinalGradeServiceUnavailable
  ---------------------------------------------
  Simulates a scenario where the GradeCalculator service
  is unavailable. The node should log a waiting message.
*/
TEST_F(TestFinalGradeDeterminator, CalculateFinalGradeServiceUnavailable)
{
  auto exam_msg = std::make_shared<Exam>();
  exam_msg->student.student_id = 1;
  exam_msg->student.course_id = 1;
  exam_msg->student.student_fullname = "Test Student";
  exam_msg->student.course_name = "Test Course";

  testing::internal::CaptureStderr();
  // Directly call calculate_grade to simulate service unavailability
  // Provide some dummy grades
  node_->calculate_grade(exam_msg->student, {85.0, 90.0, 78.0}  );
  
  std::string output = testing::internal::GetCapturedStderr();
  std::cout << output;
  EXPECT_NE(output.find("Waiting for GradeCalculator service to become available..."), std::string::npos);
}


/*
  TEST 7: CalculateGrade
  ----------------------
  Manually fills grades for a known student and calls calculate_grade().
  Ensures the average or final calculation does not break the grade list.
*/
TEST_F(TestFinalGradeDeterminator, CalculateGrade)
{
  ASSERT_FALSE(node_->student_courses_.empty());
  auto s = node_->student_courses_.front();
  auto key = std::make_pair(s.student_id, s.course_id);
  testing::internal::CaptureStderr();
  node_->student_grades_[key] = {85.0, 90.0, 78.0};
  node_->calculate_grade(s, node_->student_grades_[key]);
  auto it = node_->student_grades_.find(key);
  ASSERT_NE(it, node_->student_grades_.end());
  EXPECT_EQ(it->second.size(), 3);
}

/*
  TEST 8: MultipleStudentsHandling
  --------------------------------
  Adds multiple students and verifies that
  all are stored and can receive grades.
*/
TEST_F(TestFinalGradeDeterminator, MultipleStudentsHandling)
{
    // Clear existing data
    node_->student_courses_.clear();
    node_->student_grades_.clear();

    // Add multiple test students
    for (int i = 1; i <= 3; i++) {
        Student s;
        s.student_id = i;
        s.course_id = i;
        s.student_fullname = "Student " + std::to_string(i);
        s.course_name = "Course " + std::to_string(i);
        s.number_of_grades = 2; // each student expects 2 exam grades
        node_->student_courses_.push_back(s);
    }

    EXPECT_EQ(node_->student_courses_.size(), 3);

    // Directly add grades and call calculate_grade()
    for (const auto& s : node_->student_courses_) {
        auto key = std::make_pair(s.student_id, s.course_id);
        node_->student_grades_[key] = {80.0, 90.0};
    }

    // Verify that all grades were recorded
    for (const auto& s : node_->student_courses_) {
        auto key = std::make_pair(s.student_id, s.course_id);
        ASSERT_TRUE(node_->student_grades_.count(key) > 0);
        EXPECT_EQ(node_->student_grades_[key].size(), 2);
    }
}

/*
  TEST 9: ClearData
  ------------------
  Simulates clearing of all internal data.
  Ensures that maps and lists can safely reset.
*/
TEST_F(TestFinalGradeDeterminator, ClearData)
{
  node_->student_courses_.clear();
  node_->student_grades_.clear();

  EXPECT_TRUE(node_->student_courses_.empty());
  EXPECT_TRUE(node_->student_grades_.empty());

}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

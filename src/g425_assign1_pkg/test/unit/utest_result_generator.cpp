#include <gtest/gtest.h>

#define TESTING_EXCLUDE_MAIN
#include "../src/ResultGenerator.cpp"

class ResultGenerator;

class TestResultGenerator : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<ResultGenerator>();
  }
  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }
  std::shared_ptr<ResultGenerator> node_;
};

TEST_F(TestResultGenerator, TestNodeCreation)
{
  // Node should exist
  ASSERT_NE(node_, nullptr);

  // Check if the node name matches what we expect
  EXPECT_EQ(std::string(node_->get_name()), std::string("ResultGenerator_node"));

  // Parameters exist and have correct default values
  EXPECT_TRUE(node_->has_parameter("EXAM_PUBLISH_INTERVAL"));
  EXPECT_TRUE(node_->has_parameter("MIN_MARK"));
  EXPECT_TRUE(node_->has_parameter("MAX_MARK"));

  EXPECT_EQ(node_->get_parameter("EXAM_PUBLISH_INTERVAL").as_int(), 2);
  EXPECT_EQ(node_->get_parameter("MIN_MARK").as_int(), 10);
  EXPECT_EQ(node_->get_parameter("MAX_MARK").as_int(), 100);

  // Publisher exists
  EXPECT_GT(node_->count_publishers("exam_results"), 0u);

  // Subscribers exist
  EXPECT_GT(node_->count_subscribers("add_students"), 0u);
  EXPECT_GT(node_->count_subscribers("remove_students"), 0u);
}

TEST_F(TestResultGenerator, AddStudent)
{
  Student student;
  student.student_fullname = "Test Student";
  student.student_id = 12345;
  student.course_name = "Test Course";
  student.course_id = 101;

  // Initially, the students_ vector should be empty
  EXPECT_TRUE(node_->students_.empty());

  // Add a student
  node_->add_student(student);

  // Now the students_ vector should contain one student
  ASSERT_EQ(node_->students_.size(), 1u);
  EXPECT_EQ(node_->students_[0].student_fullname, "Test Student");
  EXPECT_EQ(node_->students_[0].student_id, 12345);
  EXPECT_EQ(node_->students_[0].course_name, "Test Course");
  EXPECT_EQ(node_->students_[0].course_id, 101);
}

TEST_F(TestResultGenerator, AddDuplicateStudent)
{
  Student student;
  student.student_fullname = "Test Student";
  student.student_id = 12345;
  student.course_name = "Test Course";
  student.course_id = 101;

    // Capture log output
  testing::internal::CaptureStderr();

  // Add a student
  node_->add_student(student);
  ASSERT_EQ(node_->students_.size(), 1u);

  // Adding the same student again should not duplicate
  node_->add_student(student);
  ASSERT_EQ(node_->students_.size(), 1u); // Still one student
  
  std::string output = testing::internal::GetCapturedStderr();
  std::cerr << output << std::flush;
  EXPECT_NE(output.find("Student Test Student (12345) for course Test Course (101) already exists. Ignored."), std::string::npos);
}

TEST_F(TestResultGenerator, RemoveStudent)
{
  Student student;
  student.student_fullname = "Test Student";
  student.student_id = 12345;
  student.course_name = "Test Course";
  student.course_id = 101;

  // Add a student first
  node_->add_student(student);
  
  ASSERT_EQ(node_->students_.size(), 1u);
  // Remove the student
  node_->remove_student(student);
  EXPECT_TRUE(node_->students_.empty());
}

TEST_F(TestResultGenerator, RemoveNonExistentStudent)
{
  Student student;
  student.student_fullname = "Test Student";
  student.student_id = 12345;
  student.course_name = "Test Course";
  student.course_id = 101;

    // Capture log output
  testing::internal::CaptureStderr();

  // Ensure students_ vector is empty
  node_->students_.clear();
  ASSERT_TRUE(node_->students_.empty());

  // Attempt to remove a non-existent student
  node_->remove_student(student);
  // The students_ vector should still be empty
  EXPECT_TRUE(node_->students_.empty());
  std::string output = testing::internal::GetCapturedStderr();
  std::cerr << output << std::flush;
  EXPECT_NE(output.find("Student Test Student (12345) for course Test Course (101) not found. Can't remove student."), std::string::npos);
}

TEST_F(TestResultGenerator, PublishRandomResultWithoutStudents)
{
  // Ensure students_ vector is empty
  node_->students_.clear();
  ASSERT_TRUE(node_->students_.empty());

  // Capture log output
  testing::internal::CaptureStderr();

  // Call publish_random_result, should warn about no students
  node_->publish_random_result();
  
  std::string output = testing::internal::GetCapturedStderr();
  std::cerr << output << std::flush;
  EXPECT_NE(output.find("No students available."), std::string::npos);
}

TEST_F(TestResultGenerator, PublishRandomResultWithStudent)
{
  Student student;
  student.student_fullname = "Test Student";
  student.student_id = 1;
  student.course_name = "Test Course";
  student.course_id = 1;

  // Add a student
  node_->add_student(student);
  ASSERT_EQ(node_->students_.size(), 1u);

  // Capture log output
  testing::internal::CaptureStderr();

  // Call publish_random_result, should publish a result
  node_->publish_random_result();

  std::string output = testing::internal::GetCapturedStderr();
  EXPECT_NE(output.find("Published result: Test Student, Course: Test Course"), std::string::npos);
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

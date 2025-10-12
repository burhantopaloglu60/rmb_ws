#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "g425_assign1_pkg/GGDatabase.hpp"

class SystemLaunchTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    db_ = new GGDatabase("localhost", "john_gradegenerator", "1234", "grade_generator");
    // specifically defining the constructor values here so you can change it to a test database while testing
  }
  void TearDown() override
  {
    db_->~GGDatabase();
  }
  GGDatabase* db_;
};

TEST_F(SystemLaunchTest, WaitForFinalGrade)
{
  bool found_final_grade = false;
  int student_id = 1;
  int course_id = 3;

  std::vector<DBT_FinalGrade> fnl_grades = db_->getAllFinalGrades();

  for (const auto& fg : fnl_grades)
  {
    if (fg.student_id == student_id && fg.course_id == course_id)
    {
      found_final_grade = true;
    }
  }
  ASSERT_TRUE(found_final_grade) << "Final grade is not in database! Run the launch file and test again!";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
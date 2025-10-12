#include <gtest/gtest.h>

// #define TESTING_EXCLUDE_MAIN # no need here
#include <g425_assign1_pkg/GGDatabase.hpp>
#include <tuple>
#include <vector>

class GGDatabase;

class TestDB : public ::testing::Test
{
protected:
  void SetUp() override
  {
    db_ = new GGDatabase("localhost","john_gradegenerator","1234","grade_generator");
    //specifically defining the constructor values here so you can change it to a test database while testing
  }
  void TearDown() override
  {
    db_->~GGDatabase();
  }
  GGDatabase* db_;
};

TEST_F(TestDB, TestSimpleFunctions)
{
  // try every query once
  EXPECT_EQ(db_->getCourseId("Mathematics"), 1);
  EXPECT_EQ(db_->getCourseName(1), "Mathematics");

  EXPECT_EQ(db_->getStudentId("Wessel Tip"), 2);
  EXPECT_EQ(db_->getStudentName(2), "Wessel Tip");

  EXPECT_EQ(db_->getGradeAmountFromCourse(db_->getCourseId("Mathematics")), 5);

  bool added_and_found_final_grade = false;
  DBT_Grade grdToAdd{0, 1, 1, 60.0};;
  DBT_FinalGrade finalGrdToAdd{0, 1, 1, 5, 60.0};;

  EXPECT_TRUE(db_->addGrade(grdToAdd));
  EXPECT_TRUE(db_->addFinalGrade(finalGrdToAdd));

  std::vector<DBT_FinalGrade> fnl_grades = db_->getAllFinalGrades();

  for (const auto& fg : fnl_grades)
  {
    if (fg.student_id == finalGrdToAdd.student_id && fg.course_id == finalGrdToAdd.course_id && //
      fg.number_of_exams == finalGrdToAdd.number_of_exams && fg.final_grade == finalGrdToAdd.final_grade)
    {
      added_and_found_final_grade = true;
    }
  }
  EXPECT_TRUE(added_and_found_final_grade);
}

TEST_F(TestDB, LongerTests)
{
  std::vector<std::tuple<int, int>> student_course_relations = db_->getAllStudentCoursesRel();
  std::vector<std::tuple<int, int>> missing_final_grades = db_->getMissingFinalGrades();


  EXPECT_GT(student_course_relations.size(), 0u);
  EXPECT_GT(missing_final_grades.size(), 0u); //this one can be difficult to test if there are no more missing final grades
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

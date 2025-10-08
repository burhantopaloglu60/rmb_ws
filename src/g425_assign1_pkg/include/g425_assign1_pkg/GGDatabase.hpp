/*
Header file for GGDatabase.cpp -Burhan Topaloglu
*/

/*
Software changes (one line by change):
(1) 29.09.2025 created and initialized by Burhan Topaloglu
(2) 03.10.2025 basic connection functionality by Burhan Topaloglu
...
*/

#ifndef __GGDATABASE_HPP__
#define __GGDATABASE_HPP__

#pragma once  // we want the database to only have one instance
#include <mariadb/mysql.h>
#include <iostream>
#include <tuple>
#include <vector>

/**
 * @brief
 * DataBase Entry for Table: students excluding value "timestamp"
 */
struct DBT_Student
{
  int id;
  std::string student_name;
  int course_id;
};

/**
 * @brief
 * DataBase Entry for Table: courses excluding value "timestamp"
 * @param number_of_exams Number of exams in this course
 */
struct DBT_Course
{
  int id;
  std::string course_name;
  int number_of_grades;
};

/**
 * @brief
 * DataBase Entry for Table: student_courses excluding value "timestamp"
 */
struct DBT_StudentCourse
{
  int student_id;  // Foreign key -> students.id
  int course_id;   // Foreign key -> courses.id
};

/**
 * @brief
 * DataBase Entry for Table: final_grades excluding value "timestamp"
 * @param number_of_exams Number of exams taken for this result
 */
struct DBT_FinalGrade
{
  int id;
  int student_id;  // Foreign key referencing students.id
  int course_id;   // Foreign key referencing courses.id
  int number_of_exams;
  double final_grade;
};

/**
 * @brief
 * DataBase Entry for Table: grades excluding value "timestamp"
 */
struct DBT_Grade
{
  int id;
  int student_id;  // Foreign key referencing students.id
  int course_id;   // Foreign key referencing courses.id
  double grade;
};

/**
 * @brief
 * Grade Generator Database
 * @param const_std::string& server
 * @param const_std::string& user
 * @param const_std::string& password
 * @param const_std::string& database
 */
class GGDatabase
{
public:
  GGDatabase(const std::string& server = "localhost", const std::string& user = "john_gradegenerator",
             const std::string& password = "1234", const std::string& database = "grade_generator");
  ~GGDatabase();

  std::tuple<bool, MYSQL*> setupConnection();
  std::string getStudentName(int s_id);
  std::string getCourseName(int c_id);
  int getStudentId(const std::string& s_name);
  int getCourseId(const std::string& c_name);
  int getGradeAmountFromCourse(int c_id);
  bool addGrade(const DBT_Grade& st_grade);
  bool addFinalGrade(const DBT_FinalGrade& st_finalGrade);
  std::vector<std::tuple<int, int>> getAllStudentCoursesRel();
  std::vector<std::tuple<int, int>> getMissingFinalGrades();
  std::vector<DBT_FinalGrade> getAllFinalGrades();

private:
  std::tuple<bool, MYSQL_RES*> execSQLQuery_(MYSQL* connection, std::string query);
  MYSQL* conn_;
  std::string server_;
  std::string user_;
  std::string password_;
  std::string database_;
};

#endif
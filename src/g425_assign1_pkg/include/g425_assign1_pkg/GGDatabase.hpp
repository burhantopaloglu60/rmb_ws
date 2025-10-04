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
// #include <mariadb/mysqld_error.h>
#include <iostream>
// #include <vector>
// #include <tuple>

/**
 * @brief
 * DataBase Entry finalgrade:
 * Use this struct to manipulate database entries, the database itself is only to be called using this struct
 */
struct DBE_finalgrade
{
  int id;
  std::string student_name;
  int student_id;
  std::string course;
  int course_id;
  int number_of_exams;
  double final_grade;
  std::string timestamp;
};

/**
 * @brief
 * Table meant for logging ALL grades
 */
struct DBE_grade
{
  int id;
  int student_id;
  int course_id;
  double grade;
  std::string timestamp;
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
  GGDatabase(const std::string& server,    // uri
             const std::string& user,      // username
             const std::string& password,  // password
             const std::string& database   // schema
  );

  std::tuple<bool, MYSQL*> SetupConnection();
  std::tuple<bool, MYSQL_RES*> ExecSQLQuery(MYSQL* connection, std::string query);

private:
  MYSQL* conn_;
  std::string server_;
  std::string user_;
  std::string password_;
  std::string database_;
};

#endif
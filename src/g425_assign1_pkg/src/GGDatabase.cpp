/*
Yes, I know plaintext passwords are bad but this is for educational purposes -Burhan

Software changes (one line by change):
(1) 29.09.2025 created and initialized by Burhan Topaloglu
(2) 03.10.2025 basic connection functionality by Burhan Topaloglu, mariadb connection basics guide by Stevesteacher
(3) 07.10.2025 added all functionality by Burhan Topaloglu
(4) 08.10.2025 simplified with Templates and finalised functionality by Burhan Topaloglu

Tested by Burhan Topaloglu on each iteration
*/

#include "g425_assign1_pkg/GGDatabase.hpp"
#include <mariadb/mysqld_error.h>
#include <sstream>
/* found in header
#include <mariadb/mysql.h>
#include <iostream>
#include <tuple>
#include <vector> */

GGDatabase::GGDatabase(const std::string& server, const std::string& user,  //
                       const std::string& password, const std::string& database)
  : server_(server), user_(user), password_(password), database_(database)
{
  bool conn_success;
  std::cout << "Connecting to MariaDB at: " << server_ << std::endl;
  std::tie(conn_success, conn_) = this->setupConnection();

  if (!conn_success)
  {
    std::cout << "connection unsuccesful" << std::endl;
  }
  std::cout << "connection succesful!" << std::endl;
}

GGDatabase::~GGDatabase()
{
  mysql_close(conn_);
}

/**
 * @brief
 * Establishes MariaDB connection, returns tuple with a boolean for success
 * @param GGDatabase() the connection is in the members of the database class
 * @return std::tuple<bool , MYSQL *>
 */
std::tuple<bool, MYSQL*> GGDatabase::setupConnection()
{
  MYSQL* connection = mysql_init(NULL);
  bool success = true;

  // if mysql_real_connect is NULL or connection is NULL
  if (!connection || !mysql_real_connect(connection,               // connection
                                         this->server_.c_str(),    // server/uri/host
                                         this->user_.c_str(),      // user
                                         this->password_.c_str(),  // password
                                         this->database_.c_str(),  // schema/db
                                         0,                        // port
                                         NULL,                     // unix_socket
                                         0                         // clientflag
                                         ))
  {
    success = false;
    std::cout << "Connection Error: " << mysql_error(connection) << std::endl;
  }
  else
  {
    conn_ = connection;
  }
  return std::make_tuple(success, connection);
}

/**
 * @brief private function for internal use to make queries easier
 * @param MYSQL* connection
 * @param std::string query
 * @return std::tuple<bool, MYSQL_RES*>
 */
std::tuple<bool, MYSQL_RES*> GGDatabase::execSQLQuery_(MYSQL* connection, std::string query)
{
  bool success = true;

  if (mysql_query(connection, query.c_str()))
  {
    std::cout << "MySQL Query Error: " << mysql_error(connection) << std::endl;
    success = false;
  }

  // mysql_store_result(con) or mysql_use_result(con)
  return std::make_tuple(success, mysql_use_result(connection));
}

#pragma region helper_templates
template <typename T>
T GGDatabase::fetchSingleValue(const std::string& query)
{
  bool result_success;
  MYSQL_RES* res;
  MYSQL_ROW row;

  std::tie(result_success, res) = execSQLQuery_(conn_, query);

  if (!result_success)
  {
    std::cout << "Query failed: " << query << std::endl;
    return T{};  // default value
  }

  T value{};
  if ((row = mysql_fetch_row(res)) != nullptr)
  {
    if constexpr (std::is_same_v<T, int>)
      value = row[0] ? std::stoi(row[0]) : 0;
    else if constexpr (std::is_same_v<T, double>)
      value = row[0] ? std::stod(row[0]) : 0.0;
    else
      value = row[0] ? row[0] : "";
  }

  mysql_free_result(res);
  return value;
}

bool GGDatabase::executeInsert(const std::string& query)
{
  bool success;
  MYSQL_RES* res;
  std::tie(success, res) = execSQLQuery_(conn_, query);
  if (!success)
    std::cout << "Insert failed: " << query << std::endl;
  mysql_free_result(res);
  return success;
}
#pragma endregion helper_templates

std::string GGDatabase::getStudentName(int s_id)
{
  return fetchSingleValue<std::string>("SELECT student_name FROM students WHERE id=" + std::to_string(s_id) + ";");
}

std::string GGDatabase::getCourseName(int c_id)
{
  return fetchSingleValue<std::string>("SELECT course_name FROM courses WHERE id=" + std::to_string(c_id) + ";");
}

int GGDatabase::getStudentId(const std::string& s_name)
{
  return fetchSingleValue<int>("SELECT id FROM students WHERE student_name='" + s_name + "';");
}

int GGDatabase::getCourseId(const std::string& c_name)
{
  return fetchSingleValue<int>("SELECT id FROM courses WHERE course_name='" + c_name + "';");
}

bool GGDatabase::addGrade(const DBT_Grade& st_grade)
{
  std::stringstream ss;
  ss << "INSERT INTO grades (student_id, course_id, grade) VALUES (" << st_grade.student_id << "," << st_grade.course_id
     << "," << st_grade.grade << ");";
  return executeInsert(ss.str());
}

bool GGDatabase::addFinalGrade(const DBT_FinalGrade& st_finalGrade)
{
  std::stringstream ss;
  ss << "INSERT INTO final_grades (student_id, course_id, number_of_exams, final_grade) VALUES ("
     << st_finalGrade.student_id << "," << st_finalGrade.course_id << "," << st_finalGrade.number_of_exams << ","
     << st_finalGrade.final_grade << ");";
  return executeInsert(ss.str());
}

int GGDatabase::getGradeAmountFromCourse(int c_id)
{
  return fetchSingleValue<int>("SELECT number_of_grades FROM courses WHERE id=" + std::to_string(c_id) + ";");
}

std::vector<std::tuple<int, int>> GGDatabase::getAllStudentCoursesRel()
{
  bool result_success;
  MYSQL_RES* res;
  std::vector<std::tuple<int, int>> result;
  MYSQL_ROW row;

  std::string q = "SELECT student_id, course_id FROM student_courses;";

  std::tie(result_success, res) = this->execSQLQuery_(conn_, q);

  if (!result_success)
  {
    std::cout << "result unsuccesful" << std::endl;
    return result;
  }

  while ((row = mysql_fetch_row(res)) != nullptr)
  {
    int student_id = row[0] ? std::stoi(row[0]) : 0;
    int course_id = row[1] ? std::stoi(row[1]) : 0;

    result.push_back(std::make_tuple(student_id, course_id));
  }

  mysql_free_result(res);
  return result;
}

std::vector<DBT_FinalGrade> GGDatabase::getAllFinalGrades()
{
  bool result_success;
  MYSQL_RES* res;
  std::vector<DBT_FinalGrade> grades;
  MYSQL_ROW row;

  std::string q = "SELECT student_id, course_id, number_of_exams, final_grade FROM final_grades;";

  std::tie(result_success, res) = this->execSQLQuery_(conn_, q);

  if (!result_success)
  {
    std::cout << "result unsuccesful" << std::endl;
    return grades;
  }

  while ((row = mysql_fetch_row(res)) != nullptr)
  {
    DBT_FinalGrade grade;
    grade.student_id = row[0] ? std::stoi(row[0]) : 0;
    grade.course_id = row[1] ? std::stoi(row[1]) : 0;
    grade.number_of_exams = row[2] ? std::stoi(row[2]) : 0;
    grade.final_grade = row[3] ? std::stod(row[3]) : 0.0;

    grades.push_back(grade);
  }

  mysql_free_result(res);
  return grades;
}

std::vector<std::tuple<int, int>> GGDatabase::getMissingFinalGrades()
{
  bool result_success;
  MYSQL_RES* res;
  std::vector<std::tuple<int, int>> result;
  MYSQL_ROW row;

  std::string q =
      "SELECT sc.* FROM student_courses sc "
      "LEFT JOIN final_grades fg ON sc.student_id = fg.student_id "
      "AND sc.course_id = fg.course_id "
      "JOIN courses c ON sc.course_id = c.id "
      "WHERE fg.id IS NULL AND c.number_of_grades <> 0;";

  std::tie(result_success, res) = this->execSQLQuery_(conn_, q);

  if (!result_success)
  {
    std::cout << "result unsuccesful" << std::endl;
    return result;
  }

  while ((row = mysql_fetch_row(res)) != nullptr)
  {
    int student_id = row[0] ? std::stoi(row[0]) : 0;
    int course_id = row[1] ? std::stoi(row[1]) : 0;

    result.push_back(std::make_tuple(student_id, course_id));
  }

  mysql_free_result(res);
  return result;
}

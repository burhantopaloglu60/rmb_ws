/*
Software changes (one line by change):
(1) 29.09.2025 created and initialized by Burhan Topaloglu
(2) 03.10.2025 basic connection functionality by Burhan Topaloglu, mariadb connection basics guided by Youtube@Stevesteacher
...
*/

/*
Let there be functions for Reading and Appending database file at "data/student_db.csv"
Burhan

CREATE DATABASE _generator;
USE grade_generator;

CREATE TABLE `final_grades` (
    `id` INT AUTO_INCREMENT PRIMARY KEY,
    `student_name` VARCHAR(100) NOT NULL,
    `course` VARCHAR(100) NOT NULL,
    `number_of_exams` INT DEFAULT 0,
    `final_grade` DECIMAL(5,2),
    `timestamp` DATETIME DEFAULT CURRENT_TIMESTAMP
);
INSERT INTO final_grades(student_name, course, number_of_exams, final_grade, timestamp) VALUES ("Sarah Connor",
"Introduction to Robotics", 2, 89.75, NOW());
*/
#include "g425_assign1_pkg/GGDatabase.hpp"
#include <mariadb/mysql.h>
#include <mariadb/mysqld_error.h>
#include <iostream>
#include <vector>
#include <tuple>

GGDatabase::GGDatabase(const std::string& server, const std::string& user, const std::string& password,
                       const std::string& database)
{
  std::cout << "Connecting to MariaDB at: " << server << std::endl;
  this->server_ = server;
  this->user_ = user;
  this->password_ = password;
  this->database_ = database;
}

/**
 * @brief
 * Establishes MariaDB connection, returns tuple with a boolean for success
 * @param GGDatabase() the connection is in the members of the database class
 * @return std::tuple<bool , MYSQL *>
 */
std::tuple<bool, MYSQL*> GGDatabase::SetupConnection()
{
  MYSQL* connection = mysql_init(NULL);
  bool success = true;

  if (!mysql_real_connect(connection,               //
                          this->server_.c_str(),    //
                          this->database_.c_str(),  //
                          this->password_.c_str(),  //
                          this->user_.c_str(), 0, NULL, 0))
  {
    success = false;
    std::cout << "Connection Error: " << mysql_error(connection) << std::endl;
  }
  return std::make_tuple(success, connection);
}

/**
 * @brief private function for internal use to make queries easier
 * @param MYSQL* connection
 * @param std::string query
 * @return struct result {
        bool success;
        MYSQL_RES *res;
    };
 */
auto execSQLQuery(MYSQL* connection, std::string query)
{
  struct result
  {
    bool success;
    MYSQL_RES* res;
  };

  bool success = true;

  if (mysql_query(connection, query.c_str()))
  {
    std::cout << "MySQL Query Error: " << mysql_error(connection) << std::endl;
    success = false;
  }

  return result{ success, mysql_use_result(connection) };
}

// mysql_free_result(result.res);
// mysql_close(con);
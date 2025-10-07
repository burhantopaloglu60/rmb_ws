/*
Let there be functions for Reading and Appending database file at "data/student_db.csv"
Yes, I know plaintext passwords are bad but this is for educational purposes -Burhan

Software changes (one line by change):
(1) 29.09.2025 created and initialized by Burhan Topaloglu
(2) 03.10.2025 basic connection functionality by Burhan Topaloglu, mariadb connection basics guided by
Youtube@Stevesteacher
...
*/

// dont forget to:
// mysql_free_result(result.res);
// mysql_close(con);

#include "g425_assign1_pkg/GGDatabase.hpp"
#include <mariadb/mysqld_error.h>
/* found in header
#include <mariadb/mysql.h>
#include <iostream>
#include <tuple> */

GGDatabase::GGDatabase(const std::string& server, const std::string& user,  //
                       const std::string& password, const std::string& database)
  : server_(std::move(server)), user_(std::move(user)), //
  password_(std::move(password)), database_(std::move(database))
{
  std::cout << "Connecting to MariaDB at: " << server_ << std::endl;
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
std::tuple<bool, MYSQL_RES*> GGDatabase::ExecSQLQuery(MYSQL* connection, std::string query)
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
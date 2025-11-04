/*
Yes, I know plaintext passwords are bad but this is for educational purposes -Burhan

Software changes:
04-11-2025 created file by using previous GGDatabase.cpp as template - Burhan Topaloglu
*/

#include "g425_assign3_pkg/ImuDatabase.hpp"
#include <mariadb/mysqld_error.h>
#include <sstream>
/* found in header
#include <mariadb/mysql.h>
#include <iostream>
#include <tuple>
#include <vector>
#include <chrono> */

ImuDatabase::ImuDatabase(const std::string& server, const std::string& user,  //
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

ImuDatabase::~ImuDatabase()
{
  mysql_close(conn_);
}

/**
 * @brief
 * Establishes MariaDB connection, returns tuple with a boolean for success
 * @param ImuDatabase() the connection is in the members of the database class
 * @return std::tuple<bool , MYSQL *>
 */
std::tuple<bool, MYSQL*> ImuDatabase::setupConnection()
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
std::tuple<bool, MYSQL_RES*> ImuDatabase::execSQLQuery_(MYSQL* connection, std::string query)
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
T ImuDatabase::fetchSingleValue(const std::string& query)
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

bool ImuDatabase::executeInsert(const std::string& query)
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

DBT_Measurement ImuDatabase::getMeasurementById(int id)
{
  bool result_success;
  MYSQL_RES* res;
  DBT_Measurement measurement;
  MYSQL_ROW row;

  std::string q =
      "SELECT id, timestamp, linear_accel_x, linear_accel_y, linear_accel_z, angular_velocity_z FROM bno055_data WHERE "
      "id = " +
      std::to_string(id) + ";";

  std::tie(result_success, res) = this->execSQLQuery_(conn_, q);

  if (!result_success)
  {
    std::cout << "result unsuccesful" << std::endl;
    return measurement;
  }

  if ((row = mysql_fetch_row(res)) != nullptr)
  {
    measurement.id = row[0] ? std::stol(row[0]) : 0;
    // Convert timestamp from string to chrono::system_clock::time_point
    if (row[1])
    {
      std::time_t ts = static_cast<std::time_t>(std::stoll(row[1]));
      measurement.timestamp = std::chrono::system_clock::from_time_t(ts);
    }
    measurement.linear_accel_x = row[2] ? std::stod(row[2]) : 0.0;
    measurement.linear_accel_y = row[3] ? std::stod(row[3]) : 0.0;
    measurement.linear_accel_z = row[4] ? std::stod(row[4]) : 0.0;
    measurement.angular_velocity_z = row[5] ? std::stod(row[5]) : 0.0;
  }

  mysql_free_result(res);
  return measurement;
}

bool ImuDatabase::addMeasurement(const DBT_Measurement& measurement)
{
  std::stringstream ss;
  // Convert timestamp to time_t for storage
  std::time_t ts = std::chrono::system_clock::to_time_t(measurement.timestamp);

  ss << "INSERT INTO bno055_data (timestamp, linear_accel_x, linear_accel_y, linear_accel_z, angular_velocity_z) "
        "VALUES ("
     << ts << "," << measurement.linear_accel_x << "," << measurement.linear_accel_y << ","
     << measurement.linear_accel_z << "," << measurement.angular_velocity_z << ");";
  return executeInsert(ss.str());
}

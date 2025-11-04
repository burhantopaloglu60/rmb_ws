/*
Header file for ImuDatabase.cpp -Burhan Topaloglu

*/

#ifndef __IMUDATABASE_HPP__
#define __IMUDATABASE_HPP__

#pragma once  // we want the database to only have one instance
#include <mariadb/mysql.h>
#include <iostream>
#include <tuple>
#include <vector>
#include <chrono>

/**
 * @brief
 * DataBase Entry for Table: "bno055_data"
 */
struct DBT_Measurement
{
  long id;
  std::chrono::system_clock::time_point timestamp; //steady clock?
  double linear_accel_x;
  double linear_accel_y;
  double linear_accel_z;
  double angular_velocity_z;
};


/**
 * @brief
 * Grade Generator Database
 * @param const_std::string& server
 * @param const_std::string& user
 * @param const_std::string& password
 * @param const_std::string& database
 */
class ImuDatabase
{
public:
  ImuDatabase(const std::string& server = "localhost", const std::string& user = "john_imu",
             const std::string& password = "1234", const std::string& database = "hello_imu");
  ~ImuDatabase();

  std::tuple<bool, MYSQL*> setupConnection();
  DBT_Measurement getMeasurementById(int id);
  bool addMeasurement(const DBT_Measurement& measurement);

private:
  std::tuple<bool, MYSQL_RES*> execSQLQuery_(MYSQL* connection, std::string query);
  template <typename T>
  T fetchSingleValue(const std::string& query);
  bool executeInsert(const std::string& query);

  MYSQL* conn_;
  std::string server_;
  std::string user_;
  std::string password_;
  std::string database_;
};

#endif

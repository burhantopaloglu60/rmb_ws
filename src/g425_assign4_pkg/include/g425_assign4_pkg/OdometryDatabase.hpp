/*
Header file for OdometryDatabase.cpp - Rik van Velzen
  25-11-2025 created file by using previous ImuDatabase.hpp as template - Rik van Velzen

*/

#ifndef __ODOMETRYDATABASE_HPP__
#define __ODOMETRYDATABASE_HPP__

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
struct DBT_Positions
{
  long id;
  std::chrono::system_clock::time_point timestamp; //steady clock?
  double x;
  double y;
  double z;
  double yaw_z;
};
struct DBT_Mecanum
{
  long id;
  std::chrono::system_clock::time_point timestamp; //steady clock?
  double wfl;
  double wfr;
  double wrl;
  double wrr;
};
/**
 * @brief
 * Grade Generator Database
 * @param const_std::string& server
 * @param const_std::string& user
 * @param const_std::string& password
 * @param const_std::string& database
 */
class OdometryDatabase
{
public:
  OdometryDatabase(const std::string& server = "localhost", const std::string& user = "john_odometry",
             const std::string& password = "1234", const std::string& database = "OdometryDB");
  ~OdometryDatabase();

  std::tuple<bool, MYSQL*> setupConnection();
  DBT_Measurement getMeasurementById(int id);
  bool addPositionmecanum(const DBT_Positions& measurement);
  bool addPositionImuSim(const DBT_Positions& measurement);
  bool addvelocitymecanum(const DBT_Mecanum measurement);
  bool addvelocityImuSim(const DBT_Measurement& measurement);
  bool addaccelerationImuSim(const DBT_Measurement& measurement);

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

cmake_minimum_required(VERSION 3.8)
project(g425_assign3_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

execute_process(COMMAND mysql_config --cflags
    OUTPUT_VARIABLE MYSQL_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND mysql_config --libs
    OUTPUT_VARIABLE MYSQL_LIBS OUTPUT_STRIP_TRAILING_WHITESPACE)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)  
find_package(sensor_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(g425_assign3_interfaces_pkg REQUIRED)

include_directories(include)

# executables
add_library(ImuDatabase src/ImuDatabase.cpp)
add_executable(udp_imu_bridge src/IMU_receiver_publisher.cpp)
add_executable(LifecycleNodeSubscriber src/Subscriber.cpp )
add_executable(ImuLifecycleNode src/ImuLifecycleNode.cpp)
add_executable(LifecycleManager src/LifecycleManager.cpp)

# dependencies
ament_target_dependencies(ImuDatabase rclcpp g425_assign3_interfaces_pkg)
ament_target_dependencies(LifecycleNodeSubscriber rclcpp g425_assign3_interfaces_pkg sensor_msgs)
ament_target_dependencies(ImuLifecycleNode rclcpp rclcpp_lifecycle sensor_msgs lifecycle_msgs)
ament_target_dependencies(LifecycleManager rclcpp lifecycle_msgs sensor_msgs)
ament_target_dependencies(udp_imu_bridge rclcpp sensor_msgs std_msgs )

# libraries
target_compile_options(ImuDatabase PRIVATE ${MYSQL_CFLAGS})
target_link_libraries(ImuDatabase ${MYSQL_LIBS})
target_link_libraries(LifecycleNodeSubscriber ImuDatabase ${MYSQL_LIBS})

# launch files
install(DIRECTORY
  launch
  config
  include
  DESTINATION share/${PROJECT_NAME}/
  )

install (TARGETS
    LifecycleNodeSubscriber
    ImuLifecycleNode
    LifecycleManager
    udp_imu_bridge
          DESTINATION lib/${PROJECT_NAME}
          )

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # test files
  ament_add_gtest(utest_imu_lifecycle_node test/unit/utest_imu_lifecycle.cpp)
  ament_add_gtest(utest_lifecycle_manager test/unit/utest_lifecycle_manager.cpp)
  ament_add_gtest(utest_subscriber test/unit/utest_subscriber.cpp)
  
  target_compile_definitions(utest_imu_lifecycle_node PRIVATE TESTING_EXCLUDE_MAIN)
  target_compile_definitions(utest_lifecycle_manager PRIVATE TESTING_EXCLUDE_MAIN)
  target_compile_definitions(utest_subscriber PRIVATE TESTING_EXCLUDE_MAIN)
  
  # dependencies
  ament_target_dependencies(utest_imu_lifecycle_node rclcpp rclcpp_lifecycle sensor_msgs lifecycle_msgs)
  ament_target_dependencies(utest_lifecycle_manager rclcpp lifecycle_msgs sensor_msgs)
  ament_target_dependencies(utest_subscriber rclcpp g425_assign3_interfaces_pkg sensor_msgs)
  
  target_link_libraries(utest_subscriber ImuDatabase ${MYSQL_LIBS})

endif()

ament_package()

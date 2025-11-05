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
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

find_package(g425_assign3_interfaces_pkg REQUIRED)

include_directories(include)

# executables
add_library(ImuDatabase src/ImuDatabase.cpp)
add_executable(ESP32Subscriber src/Subscriber.cpp )
add_executable(ImuLifecycleNode src/ImuLifecycleNode.cpp)
add_executable(LifecycleManager src/LifecycleManager.cpp)

# dependencies
ament_target_dependencies(ImuDatabase rclcpp g425_assign3_interfaces_pkg)
ament_target_dependencies(ESP32Subscriber rclcpp g425_assign3_interfaces_pkg sensor_msgs)
ament_target_dependencies(ImuLifecycleNode rclcpp rclcpp_lifecycle sensor_msgs lifecycle_msgs)
ament_target_dependencies(LifecycleManager rclcpp lifecycle_msgs sensor_msgs)

# libraries
target_compile_options(ImuDatabase PRIVATE ${MYSQL_CFLAGS})
target_link_libraries(ImuDatabase ${MYSQL_LIBS})
# target_link_libraries(NODE ImuDatabase ${MYSQL_LIBS})
target_link_libraries(ESP32Subscriber ImuDatabase ${MYSQL_LIBS})


# launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
  )

install (TARGETS
    ESP32Subscriber
    ImuLifecycleNode
    LifecycleManager
          DESTINATION lib/${PROJECT_NAME}
          )

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # test files
  # ament_add_gtest(${PROJECT_NAME}_utest_grade_calculator test/unit/utest_grade_calculator.cpp)
  # ament_add_gtest(${PROJECT_NAME}_utest_final_grade_determinator test/unit/utest_final_grade_determinator.cpp)
  # ament_add_gtest(${PROJECT_NAME}_utest_result_generator test/unit/utest_result_generator.cpp)
  # ament_add_gtest(${PROJECT_NAME}_utest_ggDatabase test/unit/utest_ggDatabase.cpp)
  # ament_add_gtest(${PROJECT_NAME}_utest_retake_scheduler test/unit/utest_retake_scheduler.cpp)
  # ament_add_gtest(${PROJECT_NAME}_utest_retake_grade_determinator test/unit/utest_retake_grade_determinator.cpp)

  # ament_add_gtest(${PROJECT_NAME}_systest_grade_generator test/systest_grade_generator.cpp)

  # target_include_directories(${PROJECT_NAME}_utest_grade_calculator PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

  # target_compile_definitions(${PROJECT_NAME}_utest_retake_scheduler PRIVATE UNIT_TEST)
  # target_compile_definitions(${PROJECT_NAME}_utest_retake_grade_determinator PRIVATE UNIT_TEST)
  
  # dependencies
  # ament_target_dependencies(${PROJECT_NAME}_utest_grade_calculator rclcpp g425_assign1_interfaces_pkg)
  # ament_target_dependencies(${PROJECT_NAME}_utest_final_grade_determinator rclcpp g425_assign1_interfaces_pkg)
  # ament_target_dependencies(${PROJECT_NAME}_utest_result_generator rclcpp g425_assign1_interfaces_pkg std_msgs)
  # ament_target_dependencies(${PROJECT_NAME}_utest_retake_scheduler rclcpp rclcpp_action g425_assign1_interfaces_pkg) 
  # ament_target_dependencies(${PROJECT_NAME}_utest_retake_grade_determinator rclcpp rclcpp_action g425_assign1_interfaces_pkg) 

  # ament_target_dependencies(${PROJECT_NAME}_systest_grade_generator rclcpp rclcpp_action g425_assign1_interfaces_pkg std_msgs)

  # target_link_libraries(${PROJECT_NAME}_utest_final_grade_determinator GGDatabase ${MYSQL_LIBS})
  # target_link_libraries(${PROJECT_NAME}_utest_result_generator GGDatabase ${MYSQL_LIBS})
  # target_link_libraries(${PROJECT_NAME}_utest_ggDatabase GGDatabase ${MYSQL_LIBS})
  # target_link_libraries(${PROJECT_NAME}_utest_retake_scheduler GGDatabase ${MYSQL_LIBS})
  # target_link_libraries(${PROJECT_NAME}_utest_retake_grade_determinator GGDatabase ${MYSQL_LIBS})

  # target_link_libraries(${PROJECT_NAME}_systest_grade_generator GGDatabase ${MYSQL_LIBS})
endif()

ament_package()

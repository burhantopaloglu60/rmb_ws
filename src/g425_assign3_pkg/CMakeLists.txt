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

find_package(g425_assign3_interfaces_pkg REQUIRED)

include_directories(include)

# executables
# add_library(GGDatabase src/GGDatabase.cpp)
add_executable(ESP32Subscriber src/Subscriber.cpp)
# add_executable(ResultGenerator src/ResultGenerator.cpp)
# add_executable(FinalGradeDeterminator src/FinalGradeDeterminator.cpp)
# add_executable(RetakeScheduler src/RetakeScheduler.cpp)
# add_executable(RetakeGradeDeterminator src/RetakeGradeDeterminator.cpp)

# dependencies
# ament_target_dependencies(GGDatabase rclcpp g425_assign1_interfaces_pkg)
ament_target_dependencies(ESP32Subscriber rclcpp g425_assign3_interfaces_pkg std_msgs )
# ament_target_dependencies(ResultGenerator rclcpp g425_assign1_interfaces_pkg std_msgs )
# ament_target_dependencies(FinalGradeDeterminator rclcpp g425_assign1_interfaces_pkg )
# ament_target_dependencies(RetakeScheduler rclcpp rclcpp_action g425_assign1_interfaces_pkg )
# ament_target_dependencies(RetakeGradeDeterminator rclcpp rclcpp_action g425_assign1_interfaces_pkg )

# libraries
# target_compile_options(GGDatabase PRIVATE ${MYSQL_CFLAGS})
# target_link_libraries(GGDatabase ${MYSQL_LIBS})
# target_link_libraries(GradeCalculator GGDatabase ${MYSQL_LIBS})
# target_link_libraries(ResultGenerator GGDatabase ${MYSQL_LIBS})
# target_link_libraries(FinalGradeDeterminator GGDatabase ${MYSQL_LIBS})
# target_link_libraries(RetakeScheduler GGDatabase ${MYSQL_LIBS})
# target_link_libraries(RetakeGradeDeterminator GGDatabase ${MYSQL_LIBS})

# launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
  )

install (TARGETS
    ESP32Subscriber
	  # GradeCalculator
    # ResultGenerator
    # FinalGradeDeterminator
    # RetakeScheduler
    # RetakeGradeDeterminator
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

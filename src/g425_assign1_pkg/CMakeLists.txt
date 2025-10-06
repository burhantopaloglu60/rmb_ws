cmake_minimum_required(VERSION 3.8)
project(g425_assign1_pkg)

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
find_package(g425_assign1_interfaces_pkg REQUIRED)

include_directories(include)
add_library(GGDatabase src/GGDatabase.cpp)
ament_target_dependencies(GGDatabase rclcpp g425_assign1_interfaces_pkg)
target_compile_options(GGDatabase PRIVATE ${MYSQL_CFLAGS})
target_link_libraries(GGDatabase ${MYSQL_LIBS})

# executables
add_executable(GradeCalculator src/GradeCalculator.cpp)
#add_executable(ResultGenerator src/ResultGenerator.cpp)
#add_executable(FinalGradeDeterminator src/FinalGradeDeterminator.cpp)
#add_executable(RetakeScheduler src/RetakeScheduler.cpp)
#add_executable(RetakeGradeDeterminator src/RetakeGradeDeterminator.cpp)

# dependencies
ament_target_dependencies(GradeCalculator rclcpp g425_assign1_interfaces_pkg )
#ament_target_dependencies(ResultGenerator rclcpp g425_assign1_interfaces_pkg )
#ament_target_dependencies(FinalGradeDeterminator rclcpp g425_assign1_interfaces_pkg )
#ament_target_dependencies(RetakeScheduler rclcpp g425_assign1_interfaces_pkg )
#ament_target_dependencies(RetakeGradeDeterminator rclcpp g425_assign1_interfaces_pkg )

# libraries
target_link_libraries(GradeCalculator GGDatabase)
#target_link_libraries(ResultGenerator GGDatabase)
#target_link_libraries(FinalGradeDeterminator GGDatabase)
#target_link_libraries(RetakeScheduler GGDatabase)
#target_link_libraries(RetakeGradeDeterminator GGDatabase)

# launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
  )

install (TARGETS
	  GradeCalculator
    #ResultGenerator
    #FinalGradeDeterminator
    #RetakeScheduler
    #RetakeGradeDeterminator
          DESTINATION lib/${PROJECT_NAME}
          )

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()

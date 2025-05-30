cmake_minimum_required(VERSION 3.8)
project(drill)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(drill_interfaces REQUIRED)

include_directories(include)

add_executable(drill_controller src/drill_controller.cpp
        include/drill/drill_controller.h
        include/drill/drill_logger.h
        src/drill_logger.cpp
        include/drill/drill_status.h
        src/drill_status.cpp

)
####
#ament_target_dependencies(drill_controller rclcpp)
ament_target_dependencies(drill_controller rclcpp rclcpp_action std_msgs sensor_msgs drill_interfaces)
####
target_include_directories(drill_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(drill_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS drill_controller
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

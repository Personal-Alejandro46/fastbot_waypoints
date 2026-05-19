cmake_minimum_required(VERSION 3.8)
project(fastbot_waypoints)
 
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
 
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(fastbot_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(fastbot_action_server src/fastbot_action_server.cpp)
ament_target_dependencies(fastbot_action_server
  rclcpp rclcpp_action geometry_msgs nav_msgs fastbot_msgs tf2)
 
install(TARGETS fastbot_action_server DESTINATION lib/${PROJECT_NAME})
 
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(waypoints_test test/waypoints_test.cpp TIMEOUT 120)
  ament_target_dependencies(waypoints_test
    rclcpp rclcpp_action nav_msgs fastbot_msgs tf2 std_srvs)
endif()
 
ament_package()


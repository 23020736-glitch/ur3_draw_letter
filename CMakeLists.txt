cmake_minimum_required(VERSION 3.8)
project(ur3_draw_letter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(draw_letter_node src/draw_letter_node.cpp)
ament_target_dependencies(draw_letter_node 
  rclcpp 
  moveit_ros_planning_interface 
  geometry_msgs 
  tf2_geometry_msgs)

install(TARGETS draw_letter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(carter_explorer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(frontier_explorer src/frontier_explorer.cpp)
ament_target_dependencies(frontier_explorer
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS
  frontier_explorer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
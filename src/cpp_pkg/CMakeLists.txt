cmake_minimum_required(VERSION 3.8)
project(cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_interface)

include_directories(include)

add_executable(simple_tf_kinematics src/simple_tf_kinematics.cpp)
ament_target_dependencies(simple_tf_kinematics rclcpp robot_interface geometry_msgs tf2 tf2_ros)

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  simple_tf_kinematics
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

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

add_executable(llama_controller src/llama_controller.cpp)
ament_target_dependencies(llama_controller rclcpp geometry_msgs)

install(
  DIRECTORY include
  DESTINATION include
)

# Enable testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  # Add the test executable
  ament_add_gtest(test_simple_tf_kinematics
    test/test_simple_tf_kinematics.cpp
  )
  target_link_libraries(test_simple_tf_kinematics
    ${rclcpp_LIBRARIES}
    ${tf2_LIBRARIES}
    ${tf2_ros_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${robot_interface_LIBRARIES}
  )
  ament_target_dependencies(test_simple_tf_kinematics
    rclcpp
    tf2
    tf2_ros
    geometry_msgs
    robot_interface
  )
endif()

install(TARGETS
  simple_tf_kinematics
  llama_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

#ifndef FRONTIER_EXPLORER_HPP_
#define FRONTIER_EXPLORER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <cmath>
#include <memory>

class FrontierExplorer : public rclcpp::Node {
public:
    FrontierExplorer();

private:
    // Callbacks
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void exploration_callback();
    
    // Core functionality
    std::vector<std::pair<double, double>> find_frontiers(const sensor_msgs::msg::LaserScan::SharedPtr& scan);
    void move_to_frontier(const std::pair<double, double>& frontier);
    bool waitForMap(const rclcpp::Duration& timeout);

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // State variables
    sensor_msgs::msg::LaserScan::SharedPtr current_scan_;
    std::vector<std::pair<double, double>> frontiers_;
    std::pair<double, double> current_goal_;
    bool has_goal_;

    // Constants
    const double GOAL_REACHED_THRESHOLD = 0.5;    // meters
    const double ANGLE_THRESHOLD = 0.1;           // radians
    const double GAP_THRESHOLD = 1.0;             // meters (minimum gap to consider as frontier)
    const double ROTATION_SPEED = 0.3;            // rad/s
    const double LINEAR_SPEED = 0.2;              // m/s
};

#endif // FRONTIER_EXPLORER_HPP_

// colcon build --packages-select frontier_explorer
// source install/setup.bash
// ros2 launch frontier_explorer complete.launch.xml
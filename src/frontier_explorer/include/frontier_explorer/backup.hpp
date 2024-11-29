#ifndef FRONTIER_EXPLORER_HPP
#define FRONTIER_EXPLORER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <utility>
#include <vector>

class FrontierExplorer : public rclcpp::Node {
public:
    FrontierExplorer();

private:
    // Constants
    static constexpr double ROTATION_SPEED = 0.5;  // rad/s

    // Callback functions
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void exploration_callback();

    // Helper functions
    std::vector<std::pair<double, double>> find_frontiers(
        const sensor_msgs::msg::LaserScan::SharedPtr& scan);
    void move_to_frontier(const std::pair<double, double>& frontier);
    bool waitForMap(const rclcpp::Duration& timeout);

    // TF2 members
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ROS publishers and subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State variables
    sensor_msgs::msg::LaserScan::SharedPtr current_scan_;
    std::vector<std::pair<double, double>> frontiers_;
    std::pair<double, double> current_goal_;
    rclcpp::Time last_goal_change_;
    bool has_goal_;
    bool rotate_left_;
    int stuck_counter_;
    double last_goal_x_;
    double last_goal_y_;
    bool last_turned_left_;
    bool first_turn_;
    rclcpp::Time last_turn_time_;
};

#endif // FRONTIER_EXPLORER_HPP
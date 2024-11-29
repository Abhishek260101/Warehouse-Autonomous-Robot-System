#ifndef FRONTIER_EXPLORER_HPP
#define FRONTIER_EXPLORER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class FrontierExplorer : public rclcpp::Node {
public:
    FrontierExplorer();

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void exploration_callback();
    void find_frontiers(const sensor_msgs::msg::LaserScan::SharedPtr& scan);

    // TF2 members
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ROS publishers and subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Current sensor data
    sensor_msgs::msg::LaserScan::SharedPtr current_scan_;
};

#endif // FRONTIER_EXPLORER_HPP
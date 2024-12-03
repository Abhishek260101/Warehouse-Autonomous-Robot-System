#ifndef FRONTIER_EXPLORER_HPP
#define FRONTIER_EXPLORER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <map>
#include <string>

class FrontierExplorer : public rclcpp::Node {
public:
    explicit FrontierExplorer(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
            .automatically_declare_parameters_from_overrides(true)
            .allow_undeclared_parameters(false));

    // Public getters and members for testing
    sensor_msgs::msg::LaserScan::SharedPtr get_current_scan() const { return current_scan_; }
    std::string get_robot_namespace() const { return robot_namespace_; }
    std::string get_map_frame() const { return map_frame_; }
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void exploration_callback();
    void find_frontiers(const sensor_msgs::msg::LaserScan::SharedPtr& scan);
    std::shared_ptr<tf2_ros::Buffer> get_tf_buffer() const { return tf_buffer_; }

private:
    bool waitForMap(const rclcpp::Duration& timeout);

    // Private members
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::LaserScan::SharedPtr current_scan_;
    std::string robot_namespace_;
    std::string map_frame_;
};

#endif // FRONTIER_EXPLORER_HPP
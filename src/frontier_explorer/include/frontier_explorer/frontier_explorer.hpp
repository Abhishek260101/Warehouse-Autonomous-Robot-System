#ifndef FRONTIER_EXPLORER_HPP
#define FRONTIER_EXPLORER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <map>
#include <string>

/**
 * @brief Main class for frontier-based exploration
 * 
 * The FrontierExplorer class implements autonomous exploration using frontier detection.
 * It processes laser scan data to identify unexplored areas (frontiers) and generates
 * movement commands to explore them.
 */
class FrontierExplorer : public rclcpp::Node {
public:
    /**
     * @brief Constructor for FrontierExplorer
     * @param options Node options for ROS2 configuration
     * 
     * Initializes the node with parameters, publishers, subscribers and timers.
     * Automatically declares parameters from overrides and disallows undeclared parameters.
     */
    explicit FrontierExplorer(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
            .automatically_declare_parameters_from_overrides(true)
            .allow_undeclared_parameters(false));

    // Public getters and testing interface
    /**
     * @brief Get the most recent laser scan data
     * @return Shared pointer to the latest LaserScan message
     */
    sensor_msgs::msg::LaserScan::SharedPtr get_current_scan() const { return current_scan_; }

    /**
     * @brief Get the robot's namespace
     * @return String containing the robot namespace
     */
    std::string get_robot_namespace() const { return robot_namespace_; }

    /**
     * @brief Get the map frame ID
     * @return String containing the map frame ID
     */
    std::string get_map_frame() const { return map_frame_; }

    /**
     * @brief Callback for processing new laser scan messages
     * @param msg Shared pointer to incoming LaserScan message
     */
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    /**
     * @brief Timer callback for exploration logic
     * 
     * Periodically checks for and moves toward frontier regions
     */
    void exploration_callback();

    /**
     * @brief Identifies frontier regions from laser scan data
     * @param scan Shared pointer to LaserScan message to analyze
     */
    void find_frontiers(const sensor_msgs::msg::LaserScan::SharedPtr& scan);

    /**
     * @brief Get the TF2 buffer used for coordinate transforms
     * @return Shared pointer to the TF2 buffer
     */
    std::shared_ptr<tf2_ros::Buffer> get_tf_buffer() const { return tf_buffer_; }

private:
    /**
     * @brief Wait for the map to become available
     * @param timeout Maximum duration to wait
     * @return true if map was received, false if timeout occurred
     */
    bool waitForMap(const rclcpp::Duration& timeout);

    // Private member variables
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;              ///< Buffer for coordinate transforms
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_; ///< Listener for TF2 transforms
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_; ///< Laser scan subscriber
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;   ///< Velocity command publisher
    rclcpp::TimerBase::SharedPtr timer_;                      ///< Timer for exploration updates
    sensor_msgs::msg::LaserScan::SharedPtr current_scan_;     ///< Most recent laser scan data
    std::string robot_namespace_;                             ///< Namespace for this robot instance
    std::string map_frame_;                                   ///< Frame ID for the map
};

#endif // FRONTIER_EXPLORER_HPP
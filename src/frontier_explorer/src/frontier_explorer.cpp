#include "frontier_explorer/frontier_explorer.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/utils.h>
#include <chrono>
#include <thread>

/**
 * @brief Constructor for the FrontierExplorer class
 * @param options Node options passed from ROS2
 * 
 * Initializes the frontier explorer node with parameters, subscribers, publishers and timers.
 * Sets up TF2 for coordinate transformations and configures topics based on robot namespace.
 */
FrontierExplorer::FrontierExplorer(const rclcpp::NodeOptions& options)
    : Node("frontier_explorer", options)
{
    // Get robot namespace from parameter without declaring it
    robot_namespace_ = this->get_parameter("robot_namespace").as_string();
    
    // Get map frame parameter
    map_frame_ = this->get_parameter("map_frame").as_string();

    RCLCPP_INFO(this->get_logger(), "Starting Frontier Explorer initialization for %s", robot_namespace_.c_str());
    
    // Configure simulation time usage
    bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
    RCLCPP_INFO(this->get_logger(), "Using sim time: %s", use_sim_time ? "true" : "false");
    
    // Set up TF2 buffer and listener for coordinate transformations
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Construct topic names using robot namespace
    std::string lidar_topic = "/" + robot_namespace_ + "/front_3d_lidar/lidar_points";
    std::string cmd_vel_topic = "/" + robot_namespace_ + "/cmd_vel";
    
    // Create subscriber for laser scan data
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        lidar_topic, 10,
        std::bind(&FrontierExplorer::scan_callback, this, std::placeholders::_1));
        
    // Create publisher for velocity commands
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        cmd_vel_topic, 10);
        
    // Create timer for periodic exploration updates
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&FrontierExplorer::exploration_callback, this));
        
    RCLCPP_INFO(this->get_logger(), "Frontier Explorer initialization complete for %s", robot_namespace_.c_str());
}

/**
 * @brief Callback function for processing incoming laser scan messages
 * @param msg Shared pointer to the laser scan message
 * 
 * Updates the current scan data and ensures correct frame ID formatting
 */
void FrontierExplorer::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    current_scan_ = msg;

    // Ensure frame_id matches robot namespace convention
    if (current_scan_->header.frame_id.empty() || 
        current_scan_->header.frame_id == robot_namespace_ + "/front_3d_lidar") {
        current_scan_->header.frame_id = robot_namespace_ + "/front_3d_lidar";
    }

    RCLCPP_DEBUG(this->get_logger(), 
                "[%s] Received scan with %zu ranges [min: %.2f, max: %.2f] in frame %s", 
                robot_namespace_.c_str(),
                msg->ranges.size(),
                msg->range_min,
                msg->range_max,
                msg->header.frame_id.c_str());
}

/**
 * @brief Main frontier detection and navigation logic
 * @param scan Shared pointer to the current laser scan data
 * 
 * Processes laser scan data to detect obstacles and frontiers, then generates
 * appropriate movement commands for exploration
 */
void FrontierExplorer::find_frontiers(const sensor_msgs::msg::LaserScan::SharedPtr& scan)
{
    if (!scan) {
        RCLCPP_WARN(this->get_logger(), "[%s] Received null scan", robot_namespace_.c_str());
        return;
    }

    try {
        // Define frame names for transformation
        std::string base_frame = "base_link";
        std::string odom_frame = "odom";

        // Verify transform availability between odom and base_link
        if (!tf_buffer_->canTransform(odom_frame, base_frame, tf2::TimePointZero)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), 
                                *this->get_clock(),
                                1000,
                                "[%s] Cannot transform between %s and %s", 
                                robot_namespace_.c_str(), 
                                odom_frame.c_str(), 
                                base_frame.c_str());
            return;
        }

        // Get current robot pose transformation
        auto transform = tf_buffer_->lookupTransform(
            map_frame_, base_frame,
            this->now(),
            rclcpp::Duration::from_seconds(1.0));
            
        // Extract robot's current heading
        double robot_yaw = tf2::getYaw(transform.transform.rotation);
        
        // Initialize variables for obstacle detection in front arc
        bool path_blocked = false;
        double min_front_distance = std::numeric_limits<double>::max();
        
        // Calculate indices for ±30° arc in front of robot
        size_t front_start_idx = (size_t)(((-M_PI/6) - scan->angle_min) / scan->angle_increment);
        size_t front_end_idx = (size_t)((M_PI/6 - scan->angle_min) / scan->angle_increment);
        
        // Variables for averaging front distance readings
        int valid_readings = 0;
        double sum_front_distance = 0.0;
        
        // Process readings in front arc
        for (size_t i = front_start_idx; i <= front_end_idx && i < scan->ranges.size(); i++) {
            if (std::isfinite(scan->ranges[i]) && 
                scan->ranges[i] > scan->range_min && 
                scan->ranges[i] < scan->range_max) {
                valid_readings++;
                sum_front_distance += scan->ranges[i];
                min_front_distance = std::min(min_front_distance, static_cast<double>(scan->ranges[i]));
                if (scan->ranges[i] < 1.2) {  // Check for obstacles within safety distance
                    path_blocked = true;
                }
            }
        }
        
        // Handle case with insufficient valid readings
        if (valid_readings < 5) {
            path_blocked = true;
            min_front_distance = valid_readings > 0 ? sum_front_distance / valid_readings : 0.0;
        }

        // Initialize movement command and state tracking variables
        geometry_msgs::msg::Twist cmd;
        static std::map<std::string, rclcpp::Time> last_direction_changes;
        static std::map<std::string, double> current_turn_directions;
        
        // Initialize robot-specific state if not already present
        if (last_direction_changes.find(robot_namespace_) == last_direction_changes.end()) {
            last_direction_changes[robot_namespace_] = this->now();
            current_turn_directions[robot_namespace_] = 1.0;
        }
        
        if (path_blocked) {
            // Stop forward motion when blocked
            cmd.linear.x = 0.0;
            
            // Find best direction to turn based on obstacle clearance
            double best_direction = 0.0;
            double max_clearance = 0.0;
            
            // Scan full 360° for best turning direction
            for (double angle = -M_PI; angle <= M_PI; angle += M_PI/6) {
                size_t idx = (size_t)((angle - scan->angle_min) / scan->angle_increment);
                if (idx < scan->ranges.size()) {
                    double distance = 0.0;
                    int valid_counts = 0;
                    
                    // Average readings around current angle
                    for (int offset = -2; offset <= 2; offset++) {
                        size_t check_idx = idx + offset;
                        if (check_idx < scan->ranges.size() && 
                            std::isfinite(scan->ranges[check_idx]) &&
                            scan->ranges[check_idx] > scan->range_min &&
                            scan->ranges[check_idx] < scan->range_max) {
                            distance += scan->ranges[check_idx];
                            valid_counts++;
                        }
                    }
                    
                    // Update best direction if better clearance found
                    if (valid_counts > 0) {
                        distance /= valid_counts;
                        if (distance > max_clearance) {
                            max_clearance = distance;
                            best_direction = angle;
                        }
                    }
                }
            }
            
            // Update turn direction periodically
            auto time_since_change = this->now() - last_direction_changes[robot_namespace_];
            if (time_since_change.seconds() > 1.0) {
                if (best_direction != 0.0) {
                    current_turn_directions[robot_namespace_] = best_direction > 0 ? 1.0 : -1.0;
                    last_direction_changes[robot_namespace_] = this->now();
                }
            }
            
            // Set turning velocity
            cmd.angular.z = 0.5 * current_turn_directions[robot_namespace_];
            
            RCLCPP_INFO(this->get_logger(), 
                "[%s] Obstacle detected at %.2fm. Turning %s", 
                robot_namespace_.c_str(),
                min_front_distance,
                current_turn_directions[robot_namespace_] > 0 ? "right" : "left");
            
        } else {
            // Path is clear - move forward with slight random turning
            cmd.linear.x = 0.2;
            double random_turn = (rand() % 100 - 50) / 1000.0;
            cmd.angular.z = random_turn;
            
            RCLCPP_INFO(this->get_logger(), 
                "[%s] Path clear. Moving forward. Distance to nearest obstacle: %.2fm",
                robot_namespace_.c_str(),
                min_front_distance);
        }
        
        // Publish movement command
        cmd_vel_pub_->publish(cmd);
        
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "[%s] Transform error: %s", 
                    robot_namespace_.c_str(), ex.what());
    }
}

/**
 * @brief Timer callback for periodic exploration updates
 * 
 * Checks for valid scan data and triggers frontier detection
 */
void FrontierExplorer::exploration_callback()
{
    if (!current_scan_) {
        RCLCPP_DEBUG(this->get_logger(), "[%s] No laser scan data received yet", 
                     robot_namespace_.c_str());
        return;
    }
    
    find_frontiers(current_scan_);
}

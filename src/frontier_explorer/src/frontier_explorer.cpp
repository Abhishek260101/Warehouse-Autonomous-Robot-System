#include "frontier_explorer/frontier_explorer.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/utils.h>
#include <chrono>
#include <thread>

FrontierExplorer::FrontierExplorer()
: Node("frontier_explorer", rclcpp::NodeOptions()
    .automatically_declare_parameters_from_overrides(true)
    .allow_undeclared_parameters(false))  
{
    RCLCPP_INFO(this->get_logger(), "Starting Frontier Explorer initialization");
    
    // Get use_sim_time parameter (should be already declared via launch file)
    bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
    RCLCPP_INFO(this->get_logger(), "Using sim time: %s", use_sim_time ? "true" : "false");
    
    // Initialize TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Initialize subscribers
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/carter1/front_3d_lidar/lidar_points", 10,
        std::bind(&FrontierExplorer::scan_callback, this, std::placeholders::_1));
        
    // Initialize publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/carter1/cmd_vel", 10);
        
    // Initialize timer for exploration with a slower rate
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),  // 2Hz instead of 1Hz
        std::bind(&FrontierExplorer::exploration_callback, this));
        
    RCLCPP_INFO(this->get_logger(), "Frontier Explorer initialization complete");
}

void FrontierExplorer::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    current_scan_ = msg;

    // Set the correct frame_id if it's not already set
    if (current_scan_->header.frame_id.empty() || 
        current_scan_->header.frame_id == "carter1/front_3d_lidar") {
        current_scan_->header.frame_id = "front_3d_lidar";
    }

    RCLCPP_DEBUG(this->get_logger(), 
                "Received scan with %zu ranges [min: %.2f, max: %.2f] in frame %s", 
                msg->ranges.size(),
                msg->range_min,
                msg->range_max,
                msg->header.frame_id.c_str());
}


void FrontierExplorer::find_frontiers(const sensor_msgs::msg::LaserScan::SharedPtr& scan)
{
    if (!scan) {
        RCLCPP_WARN(this->get_logger(), "Received null scan");
        return;
    }

    try {
        auto transform = tf_buffer_->lookupTransform(
            "map", "base_link", 
            this->now(),
            rclcpp::Duration::from_seconds(1.0));
            
        double robot_yaw = tf2::getYaw(transform.transform.rotation);
        
        // Increased forward arc to ±30° (from ±15°)
        bool path_blocked = false;
        double min_front_distance = std::numeric_limits<double>::max();
        
        size_t front_start_idx = (size_t)(((-M_PI/6) - scan->angle_min) / scan->angle_increment);
        size_t front_end_idx = (size_t)((M_PI/6 - scan->angle_min) / scan->angle_increment);
        
        // Count valid readings
        int valid_readings = 0;
        double sum_front_distance = 0.0;
        
        for (size_t i = front_start_idx; i <= front_end_idx && i < scan->ranges.size(); i++) {
            if (std::isfinite(scan->ranges[i]) && scan->ranges[i] > scan->range_min && scan->ranges[i] < scan->range_max) {
                valid_readings++;
                sum_front_distance += scan->ranges[i];
                min_front_distance = std::min(min_front_distance, static_cast<double>(scan->ranges[i]));
                if (scan->ranges[i] < 1.2) {  // Increased safety distance from 0.8m to 1.2m
                    path_blocked = true;
                }
            }
        }
        
        // If we don't have enough valid readings, consider path blocked
        if (valid_readings < 5) {
            path_blocked = true;
            min_front_distance = valid_readings > 0 ? sum_front_distance / valid_readings : 0.0;
        }

        geometry_msgs::msg::Twist cmd;
        static rclcpp::Time last_direction_change = this->now();
        static double current_turn_direction = 1.0;  // 1.0 for right, -1.0 for left
        
        if (path_blocked) {
            // Stop forward motion when obstacle detected
            cmd.linear.x = 0.0;
            
            // Scan surroundings in 30-degree increments to reduce oscillation
            double best_direction = 0.0;
            double max_clearance = 0.0;
            
            // Check both left and right sides with wider angles
            for (double angle = -M_PI; angle <= M_PI; angle += M_PI/6) {  // 30-degree increments
                size_t idx = (size_t)((angle - scan->angle_min) / scan->angle_increment);
                if (idx < scan->ranges.size()) {
                    double distance = 0.0;
                    int valid_counts = 0;
                    
                    // Average over a small window to get more stable readings
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
                    
                    if (valid_counts > 0) {
                        distance /= valid_counts;
                        if (distance > max_clearance) {
                            max_clearance = distance;
                            best_direction = angle;
                        }
                    }
                }
            }
            
            // Prevent rapid direction changes by maintaining direction for at least 1 second
            auto time_since_change = this->now() - last_direction_change;
            if (time_since_change.seconds() > 1.0) {
                if (best_direction != 0.0) {
                    current_turn_direction = best_direction > 0 ? 1.0 : -1.0;
                    last_direction_change = this->now();
                }
            }
            
            // Use the maintained direction with a fixed turning speed
            cmd.angular.z = 0.5 * current_turn_direction;
            
            RCLCPP_INFO(this->get_logger(), 
                "Obstacle detected at %.2fm. Turning %s", 
                min_front_distance,
                current_turn_direction > 0 ? "right" : "left");
            
        } else {
            // Path is clear - move forward with slight random exploration
            cmd.linear.x = 0.2;  // Reduced speed for safety
            
            // Add small random rotation for exploration
            double random_turn = (rand() % 100 - 50) / 1000.0;  // Reduced random factor
            cmd.angular.z = random_turn;
            
            RCLCPP_INFO(this->get_logger(), 
                "Path clear. Moving forward. Distance to nearest obstacle: %.2fm",
                min_front_distance);
        }
        
        cmd_vel_pub_->publish(cmd);
        
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
    }
}

void FrontierExplorer::exploration_callback()
{
    if (!current_scan_) {
        RCLCPP_DEBUG(this->get_logger(), "No laser scan data received yet");
        return;
    }
    
    find_frontiers(current_scan_);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrontierExplorer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
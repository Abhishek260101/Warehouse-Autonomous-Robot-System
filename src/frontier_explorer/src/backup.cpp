#include "frontier_explorer/frontier_explorer.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/utils.h>
#include <chrono>
#include <thread>

FrontierExplorer::FrontierExplorer()
: Node("frontier_explorer", rclcpp::NodeOptions()
    .automatically_declare_parameters_from_overrides(true)
    .allow_undeclared_parameters(false)),
    last_goal_change_(this->get_clock()->now()),
    has_goal_(false),
    rotate_left_(true),
    stuck_counter_(0),
    last_goal_x_(0.0),
    last_goal_y_(0.0),
    last_turned_left_(false),
    first_turn_(true),
    last_turn_time_(this->now())    
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

bool FrontierExplorer::waitForMap(const rclcpp::Duration& timeout)
{
    auto start_time = this->now();
    while (rclcpp::ok()) {
        if (tf_buffer_->_frameExists("map")) {
            RCLCPP_INFO(this->get_logger(), "Map frame became available");
            return true;
        }
        if ((this->now() - start_time) > timeout) {
            RCLCPP_WARN(this->get_logger(), "Timeout waiting for map frame");
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
}

std::vector<std::pair<double, double>> FrontierExplorer::find_frontiers(
    const sensor_msgs::msg::LaserScan::SharedPtr& scan)
{
    std::vector<std::pair<double, double>> frontiers;
    
    if (!tf_buffer_->_frameExists("map")) {
        if (!waitForMap(rclcpp::Duration::from_seconds(5.0))) {
            return frontiers;
        }
    }
    
    try {
        rclcpp::Time transform_time = this->get_parameter("use_sim_time").as_bool() ?
            rclcpp::Time(scan->header.stamp) : this->now();

        // Get robot's transform
        auto transform = tf_buffer_->lookupTransform(
            "map", "base_link", 
            transform_time,
            rclcpp::Duration::from_seconds(1.0));
            
        double robot_yaw = tf2::getYaw(transform.transform.rotation);
        
        // Check forward path for immediate obstacles
        double min_front_distance = std::numeric_limits<double>::max();
        bool imminent_collision = false;
        
        // Check -15° to +15° for obstacles within 0.5 meters
        for (size_t i = 0; i < scan->ranges.size(); i++) {
            double angle = scan->angle_min + i * scan->angle_increment;
            if (angle >= -M_PI/12 && angle <= M_PI/12) {  // ±15 degrees
                if (std::isfinite(scan->ranges[i])) {
                    min_front_distance = std::min(min_front_distance, static_cast<double>(scan->ranges[i]));
                    if (scan->ranges[i] < 0.5) {
                        imminent_collision = true;
                    }
                }
            }
        }
        
        // If no imminent collision, set frontier point straight ahead
        if (!imminent_collision) {
            double forward_distance = 5.0;  // Look 2 meters ahead
            double goal_x = transform.transform.translation.x + 
                forward_distance * cos(robot_yaw);
            double goal_y = transform.transform.translation.y + 
                forward_distance * sin(robot_yaw);
                
            frontiers.push_back({goal_x, goal_y});
            RCLCPP_INFO(this->get_logger(), "Path clear, moving forward. Nearest obstacle: %.2f m", min_front_distance);
            return frontiers;
        }
        
        // If obstacle detected, check left and right for clearance
        double left_min = std::numeric_limits<double>::max();
        double right_min = std::numeric_limits<double>::max();
        
        for (size_t i = 0; i < scan->ranges.size(); i++) {
            double angle = scan->angle_min + i * scan->angle_increment;
            if (std::isfinite(scan->ranges[i])) {
                if (angle > 0 && angle <= M_PI/6) {  // 0° to 30° (right)
                    right_min = std::min(right_min, static_cast<double>(scan->ranges[i]));
                } else if (angle < 0 && angle >= -M_PI/6) {  // -30° to 0° (left)
                    left_min = std::min(left_min, static_cast<double>(scan->ranges[i]));
                }
            }
        }
        
        // Turn in the direction with more space
        double turn_angle;
        if (left_min > right_min && left_min > 0.5) {
            turn_angle = M_PI/9;  // 20 degrees left
            RCLCPP_INFO(this->get_logger(), "Obstacle close (%.2f m), turning left", min_front_distance);
        } else if (right_min > 0.5) {
            turn_angle = -M_PI/9;  // 20 degrees right
            RCLCPP_INFO(this->get_logger(), "Obstacle close (%.2f m), turning right", min_front_distance);
        } else {
            // Both sides blocked, turn more sharply
            turn_angle = (left_min > right_min) ? M_PI/4 : -M_PI/4;  // 45 degrees
            RCLCPP_INFO(this->get_logger(), "Confined space, sharp turn %s", (left_min > right_min) ? "left" : "right");
        }
        
        double turn_distance = 2.0;  // 1 meter ahead after turn
        double goal_x = transform.transform.translation.x + 
            turn_distance * cos(robot_yaw + turn_angle);
        double goal_y = transform.transform.translation.y + 
            turn_distance * sin(robot_yaw + turn_angle);
            
        frontiers.push_back({goal_x, goal_y});
        
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
    }
    
    return frontiers;
}

void FrontierExplorer::move_to_frontier(const std::pair<double, double>& frontier)
{
    try {
        rclcpp::Time transform_time = this->get_parameter("use_sim_time").as_bool() ?
            this->now() : this->now();
            
        auto transform = tf_buffer_->lookupTransform(
            "map", "base_link", 
            transform_time,
            rclcpp::Duration::from_seconds(1.0));

        geometry_msgs::msg::Twist cmd;
        
        // Calculate relative position of frontier
        double dx = frontier.first - transform.transform.translation.x;
        double dy = frontier.second - transform.transform.translation.y;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        // Calculate angle to target
        double target_angle = std::atan2(dy, dx);
        double current_angle = tf2::getYaw(transform.transform.rotation);
        double angle_diff = target_angle - current_angle;
        
        // Normalize angle
        while (angle_diff > M_PI) angle_diff -= 2*M_PI;
        while (angle_diff < -M_PI) angle_diff += 2*M_PI;

        if (distance < 0.1) {  // Goal reached threshold
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            has_goal_ = false;
        } else {
            // Proportional control for smooth movement
            if (std::abs(angle_diff) > 0.1) {  // 0.1 radians ≈ 5.7 degrees
                // Turn to face target
                cmd.linear.x = 0.0;
                cmd.angular.z = std::copysign(std::min(0.5, std::abs(angle_diff)), angle_diff);
            } else {
                // Move towards target
                cmd.linear.x = std::min(0.2, distance);
                cmd.angular.z = 0.2 * angle_diff;  // Small corrections while moving
            }
        }
        
        cmd_vel_pub_->publish(cmd);
        
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform robot pose: %s", ex.what());
    }
}

void FrontierExplorer::exploration_callback()
{
    if (!current_scan_) {
        RCLCPP_DEBUG(this->get_logger(), "No laser scan data received yet");
        return;
    }
    
    frontiers_ = find_frontiers(current_scan_);
    
    if (frontiers_.empty()) {
        geometry_msgs::msg::Twist cmd;
        cmd.angular.z = ROTATION_SPEED;
        cmd_vel_pub_->publish(cmd);
        has_goal_ = false;
        RCLCPP_DEBUG(this->get_logger(), "No frontiers found, rotating to scan");
        return;
    }
    
    if (!has_goal_) {
        try {
            // Get transform at latest available time
            auto transform = tf_buffer_->lookupTransform(
                "map",
                "base_link",
                this->get_clock()->now(),  // Current time
                rclcpp::Duration::from_seconds(1.0));
                                     
            double robot_x = transform.transform.translation.x;
            double robot_y = transform.transform.translation.y;
            
            // Find closest frontier
            double min_dist = std::numeric_limits<double>::max();
            size_t closest_idx = 0;
            
            for (size_t i = 0; i < frontiers_.size(); ++i) {
                double dx = frontiers_[i].first - robot_x;
                double dy = frontiers_[i].second - robot_y;
                double dist = std::sqrt(dx*dx + dy*dy);
                
                if (dist < min_dist) {
                    min_dist = dist;
                    closest_idx = i;
                }
            }
            
            current_goal_ = frontiers_[closest_idx];
            has_goal_ = true;
            
            RCLCPP_INFO(this->get_logger(), "New goal selected at (%f, %f), distance: %f",
                       current_goal_.first, current_goal_.second, min_dist);
            
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get robot position: %s", ex.what());
        }
    }
    
    if (has_goal_) {
        move_to_frontier(current_goal_);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrontierExplorer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
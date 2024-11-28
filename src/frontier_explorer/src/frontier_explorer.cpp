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
    has_goal_(false)
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
    
    // First check if map exists
    if (!tf_buffer_->_frameExists("map")) {
        if (!waitForMap(rclcpp::Duration::from_seconds(5.0))) {
            RCLCPP_DEBUG(this->get_logger(), "Still waiting for map frame...");
            return frontiers;
        }
    }
    
    try {
        rclcpp::Time transform_time;
        
        // Use appropriate timestamp
        if (this->get_parameter("use_sim_time").as_bool()) {
            transform_time = rclcpp::Time(scan->header.stamp);
        } else {
            transform_time = this->now();
        }

        // Check transform availability
        if (!tf_buffer_->canTransform(
            "map",
            "front_3d_lidar",
            transform_time,
            rclcpp::Duration::from_seconds(1.0)))
        {
            RCLCPP_DEBUG(this->get_logger(), 
                "Transform not available for time %f", 
                transform_time.seconds());
            return frontiers;
        }

        // Get transform from laser to map frame
        geometry_msgs::msg::TransformStamped transform = 
            tf_buffer_->lookupTransform(
                "map",
                "front_3d_lidar",
                transform_time,
                rclcpp::Duration::from_seconds(1.0));
        
        std::vector<std::pair<double, double>> points;
        std::vector<std::pair<double, double>> obstacles;
        double angle = scan->angle_min;
        int valid_points = 0;
        
        // Convert scan to points, limiting to -45° to +45°
        for (size_t i = 0; i < scan->ranges.size(); i++) {
            if (angle >= -M_PI/4 && angle <= M_PI/4) {  // -45° to +45°
                const auto& r = scan->ranges[i];
                if (std::isfinite(r) && r >= scan->range_min && r <= scan->range_max) {
                    // Convert to point in laser frame
                    geometry_msgs::msg::PointStamped point_laser;
                    point_laser.header = scan->header;
                    point_laser.point.x = r * cos(angle);
                    point_laser.point.y = r * sin(angle);
                    point_laser.point.z = 0.0;
                    
                    // Transform to map frame
                    geometry_msgs::msg::PointStamped point_map;
                    tf2::doTransform(point_laser, point_map, transform);
                    
                    points.push_back({point_map.point.x, point_map.point.y});
                    
                    // Track obstacles (points closer than safe distance)
                    if (r < 1.0) {  // 1 meter safe distance
                        obstacles.push_back({point_map.point.x, point_map.point.y});
                    }
                    
                    valid_points++;
                }
            }
            angle += scan->angle_increment;
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Processed %d valid points", valid_points);
        
        // Find gaps between points that could be frontiers
        if (points.size() >= 2) {
            for (size_t i = 0; i < points.size() - 1; ++i) {
                double dx = points[i+1].first - points[i].first;
                double dy = points[i+1].second - points[i].second;
                double distance = std::sqrt(dx*dx + dy*dy);
                
                if (distance > GAP_THRESHOLD) {
                    double frontier_x = (points[i+1].first + points[i].first) / 2.0;
                    double frontier_y = (points[i+1].second + points[i].second) / 2.0;
                    
                    // Check if frontier is accessible
                    bool is_accessible = true;
                    
                    // Check distance from obstacles
                    for (const auto& obstacle : obstacles) {
                        double obs_dx = frontier_x - obstacle.first;
                        double obs_dy = frontier_y - obstacle.second;
                        double obs_dist = std::sqrt(obs_dx*obs_dx + obs_dy*obs_dy);
                        
                        if (obs_dist < 1.0) {  // Too close to obstacle
                            is_accessible = false;
                            break;
                        }
                    }
                    
                    // Check if this frontier is too close to existing frontiers
                    for (const auto& existing : frontiers) {
                        double f_dx = frontier_x - existing.first;
                        double f_dy = frontier_y - existing.second;
                        double f_dist = std::sqrt(f_dx*f_dx + f_dy*f_dy);
                        
                        if (f_dist < 0.5) {  // Too close to existing frontier
                            is_accessible = false;
                            break;
                        }
                    }
                    
                    // Only add accessible frontiers
                    if (is_accessible) {
                        frontiers.push_back({frontier_x, frontier_y});
                        RCLCPP_DEBUG(this->get_logger(), 
                            "Found frontier at (%f, %f)", frontier_x, frontier_y);
                    }
                }
            }
        }
        
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
    }
    
    return frontiers;
}

void FrontierExplorer::move_to_frontier(const std::pair<double, double>& frontier)
{
    try {
        auto transform = tf_buffer_->lookupTransform(
            "map", "base_link", 
            this->get_clock()->now(),
            rclcpp::Duration::from_seconds(1.0));

        geometry_msgs::msg::Twist cmd;
        bool obstacle_detected = false;
        double min_front_dist = std::numeric_limits<double>::max();

        if (current_scan_) {
            // Check front sector for obstacles
            double angle = SCAN_ANGLE_MIN;
            for (size_t i = 0; i < current_scan_->ranges.size(); i++) {
                if (angle >= SCAN_ANGLE_MIN && angle <= SCAN_ANGLE_MAX) {
                    if (std::isfinite(current_scan_->ranges[i])) {
                        min_front_dist = std::min(min_front_dist, 
                            static_cast<double>(current_scan_->ranges[i]));
                    }
                }
                angle += current_scan_->angle_increment;
                if (angle > SCAN_ANGLE_MAX) break;
            }

            obstacle_detected = min_front_dist < 1.0;
        }

        double dx = frontier.first - transform.transform.translation.x;
        double dy = frontier.second - transform.transform.translation.y;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        if (distance < GOAL_REACHED_THRESHOLD) {
            has_goal_ = false;
            cmd.linear.x = cmd.angular.z = 0.0;
        } else if (obstacle_detected) {
            has_goal_ = false;  // Force new goal selection
            cmd.linear.x = -0.1;  // Back up slightly
            cmd.angular.z = 0.3;  // Turn to find new path
        } else {
            double robot_yaw = tf2::getYaw(transform.transform.rotation);
            double target_angle = std::atan2(dy, dx);
            double angle_diff = target_angle - robot_yaw;
            
            // Normalize angle
            while (angle_diff > M_PI) angle_diff -= 2*M_PI;
            while (angle_diff < -M_PI) angle_diff += 2*M_PI;

            if (std::abs(angle_diff) > 0.1) {
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.3 * angle_diff;
            } else {
                cmd.linear.x = 0.2;
                cmd.angular.z = 0.1 * angle_diff;  // Small correction while moving
            }
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
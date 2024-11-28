#include "frontier_explorer/frontier_explorer.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/utils.h>
#include <chrono>
#include <thread>

FrontierExplorer::FrontierExplorer()
: Node("frontier_explorer"), has_goal_(false)
{
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
        
    // Initialize timer for exploration
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&FrontierExplorer::exploration_callback, this));
        
    RCLCPP_INFO(this->get_logger(), "Frontier Explorer initialized");
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
        // Check if transform is available
        if (!tf_buffer_->canTransform(
            "map",
            "front_3d_lidar",
            scan->header.stamp,
            rclcpp::Duration::from_seconds(1.0)))
        {
            RCLCPP_DEBUG(this->get_logger(), 
                "Transform from %s to map not available yet", 
                scan->header.frame_id.c_str());
            return frontiers;
        }

        // Get transform from laser to map frame
        geometry_msgs::msg::TransformStamped transform = 
            tf_buffer_->lookupTransform(
                "map",
                "front_3d_lidar",
                scan->header.stamp,
                rclcpp::Duration::from_seconds(1.0));
        
        std::vector<std::pair<double, double>> points;
        double angle = scan->angle_min;
        int valid_points = 0;
        
        // Convert scan to points in map frame
        for (const auto& r : scan->ranges) {
            if (std::isfinite(r) && r >= scan->range_min && r <= scan->range_max) {
                geometry_msgs::msg::PointStamped point_laser;
                point_laser.header = scan->header;
                point_laser.point.x = r * cos(angle);
                point_laser.point.y = r * sin(angle);
                point_laser.point.z = 0.0;
                
                geometry_msgs::msg::PointStamped point_map;
                tf2::doTransform(point_laser, point_map, transform);
                
                points.push_back({point_map.point.x, point_map.point.y});
                valid_points++;
            }
            angle += scan->angle_increment;
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Processed %d valid points from scan", valid_points);
        
        // Find gaps between points that could be frontiers
        if (points.size() >= 2) {
            for (size_t i = 0; i < points.size() - 1; ++i) {
                double dx = points[i+1].first - points[i].first;
                double dy = points[i+1].second - points[i].second;
                double distance = std::sqrt(dx*dx + dy*dy);
                
                if (distance > GAP_THRESHOLD) {
                    double frontier_x = (points[i+1].first + points[i].first) / 2.0;
                    double frontier_y = (points[i+1].second + points[i].second) / 2.0;
                    frontiers.push_back({frontier_x, frontier_y});
                }
            }
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Found %zu frontiers", frontiers.size());
        
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform laser scan: %s", ex.what());
    }
    
    return frontiers;
}

void FrontierExplorer::move_to_frontier(const std::pair<double, double>& frontier)
{
    try {
        // Get robot's current position in map frame
        geometry_msgs::msg::TransformStamped transform = 
            tf_buffer_->lookupTransform(
                "map",
                "base_link",
                this->now(),
                rclcpp::Duration::from_seconds(1.0));
                                     
        // Calculate relative position of frontier
        double dx = frontier.first - transform.transform.translation.x;
        double dy = frontier.second - transform.transform.translation.y;
        
        // Convert to robot's frame
        double robot_yaw = tf2::getYaw(transform.transform.rotation);
        double rel_x = dx * cos(-robot_yaw) - dy * sin(-robot_yaw);
        double rel_y = dx * sin(-robot_yaw) + dy * cos(-robot_yaw);
        
        double angle_to_goal = std::atan2(rel_y, rel_x);
        double distance = std::sqrt(rel_x*rel_x + rel_y*rel_y);
        
        geometry_msgs::msg::Twist cmd;
        
        if (distance < GOAL_REACHED_THRESHOLD) {
            has_goal_ = false;
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
        }
        else if (std::abs(angle_to_goal) > ANGLE_THRESHOLD) {
            cmd.linear.x = 0.0;
            cmd.angular.z = (angle_to_goal > 0) ? ROTATION_SPEED : -ROTATION_SPEED;
            RCLCPP_DEBUG(this->get_logger(), "Turning to frontier");
        }
        else {
            cmd.linear.x = LINEAR_SPEED;
            cmd.angular.z = 0.0;
            RCLCPP_DEBUG(this->get_logger(), "Moving to frontier");
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
            // Get robot's current position in map frame
            geometry_msgs::msg::TransformStamped transform = 
                tf_buffer_->lookupTransform(
                    "map",
                    "base_link",
                    this->now(),
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
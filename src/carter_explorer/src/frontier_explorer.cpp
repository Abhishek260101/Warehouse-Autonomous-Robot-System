#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <vector>
#include <queue>
#include <cmath>
#include <memory>

class FrontierExplorer : public rclcpp::Node {
public:
    FrontierExplorer() : Node("frontier_explorer") {
        // Read launch file parameters first
        this->declare_parameter("robot_name", "carter1");
        this->declare_parameter("min_frontier_size", 10);
        this->declare_parameter("exploration_timeout", 300.0);
        this->declare_parameter("linear_velocity", 0.2);
        this->declare_parameter("angular_velocity", 0.3);
        this->declare_parameter("goal_tolerance", 0.5);
        this->declare_parameter("map_update_tolerance", 5.0);
        this->declare_parameter("rotation_duration", 12.0);

        robot_name_ = this->get_parameter("robot_name").as_string();
        min_frontier_size_ = this->get_parameter("min_frontier_size").as_int();
        
        RCLCPP_INFO(this->get_logger(), "Starting Frontier Explorer for %s", robot_name_.c_str());

        // Initialize TF buffer and listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/" + robot_name_ + "/cmd_vel", 10);

        // Create subscribers with reliable QoS
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
            .reliable()
            .durability_volatile();

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", qos, std::bind(&FrontierExplorer::mapCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/" + robot_name_ + "/chassis/odom", qos, 
            std::bind(&FrontierExplorer::odomCallback, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/" + robot_name_ + "/front_2d_lidar/scan", qos,
            std::bind(&FrontierExplorer::scanCallback, this, std::placeholders::_1));

        // Create timers with slightly different periods to avoid synchronization issues
        explore_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&FrontierExplorer::exploreLoop, this));

        status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1100),  // Slightly offset from 1 second
            std::bind(&FrontierExplorer::printStatus, this));

        // Initialize exploration
        current_state_ = INITIALIZING;
        start_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Frontier Explorer initialized and waiting for data");
    }

private:
    enum ExplorationState {
        INITIALIZING,
        SCANNING,
        MOVING_TO_FRONTIER,
        ROTATING,
        FINISHED
    };

    struct Point {
        int x, y;
        Point(int x_, int y_) : x(x_), y(y_) {}
    };

    struct Frontier {
        std::vector<Point> points;
        Point centroid;
        Frontier() : centroid(0, 0) {}
    };

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        if (!map_received_) {
            RCLCPP_INFO(this->get_logger(), 
                "Received first map update - Size: %dx%d, Resolution: %.3f",
                msg->info.width, msg->info.height, msg->info.resolution);
        }
        map_ = *msg;
        map_received_ = true;
        checkInitialization();
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!pose_received_) {
            RCLCPP_INFO(this->get_logger(), "Received first odometry message");
        }
        current_pose_ = *msg;
        pose_received_ = true;
        checkInitialization();
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (!scan_received_) {
            RCLCPP_INFO(this->get_logger(), "Received first laser scan");
        }
        latest_scan_ = *msg;
        scan_received_ = true;
        checkInitialization();
    }

    void checkInitialization() {
        if (current_state_ == INITIALIZING && map_received_ && pose_received_ && scan_received_) {
            RCLCPP_INFO(this->get_logger(), "All data received, starting exploration");
            current_state_ = SCANNING;
        }
    }

    void printStatus() {
        RCLCPP_INFO(this->get_logger(), 
            "Status - Map: %s, Odom: %s, Scan: %s, State: %d", 
            map_received_ ? "✓" : "✗",
            pose_received_ ? "✓" : "✗",
            scan_received_ ? "✓" : "✗",
            current_state_);

        if (map_received_) {
            int known_cells = 0;
            int free_cells = 0;
            for (const auto& cell : map_.data) {
                if (cell >= 0) known_cells++;
                if (cell == 0) free_cells++;
            }
            float explored_percentage = (float)known_cells / map_.data.size() * 100;
            RCLCPP_INFO(this->get_logger(), 
                "Exploration progress: %.1f%% (Free: %.1f%%)",
                explored_percentage,
                (float)free_cells / map_.data.size() * 100);
        }
    }

    bool isFrontierPoint(int x, int y) {
        // Check bounds
        if (x < 0 || x >= (int)map_.info.width || y < 0 || y >= (int)map_.info.height) {
            return false;
        }

        // Point must be unknown (-1)
        if (map_.data[y * map_.info.width + x] != -1) {
            return false;
        }

        // Must have at least one free neighbor
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) continue;
                
                int nx = x + dx;
                int ny = y + dy;
                
                if (nx >= 0 && nx < (int)map_.info.width &&
                    ny >= 0 && ny < (int)map_.info.height &&
                    map_.data[ny * map_.info.width + nx] == 0) {
                    return true;
                }
            }
        }
        return false;
    }

    std::vector<Frontier> detectFrontiers() {
        std::vector<Frontier> frontiers;
        std::vector<std::vector<bool>> visited(
            map_.info.height, std::vector<bool>(map_.info.width, false));

        // Convert robot position to grid coordinates
        int robot_x = (current_pose_.pose.pose.position.x - map_.info.origin.position.x) 
                     / map_.info.resolution;
        int robot_y = (current_pose_.pose.pose.position.y - map_.info.origin.position.y) 
                     / map_.info.resolution;

        // Flood fill from robot position
        std::queue<Point> queue;
        queue.push(Point(robot_x, robot_y));
        visited[robot_y][robot_x] = true;

        while (!queue.empty()) {
            Point p = queue.front();
            queue.pop();

            if (isFrontierPoint(p.x, p.y)) {
                Frontier frontier;
                std::queue<Point> frontier_queue;
                frontier_queue.push(p);
                visited[p.y][p.x] = true;

                // Collect connected frontier points
                while (!frontier_queue.empty()) {
                    Point fp = frontier_queue.front();
                    frontier_queue.pop();
                    frontier.points.push_back(fp);

                    for (int dx = -1; dx <= 1; dx++) {
                        for (int dy = -1; dy <= 1; dy++) {
                            int nx = fp.x + dx;
                            int ny = fp.y + dy;
                            if (nx >= 0 && nx < (int)map_.info.width &&
                                ny >= 0 && ny < (int)map_.info.height &&
                                !visited[ny][nx] && isFrontierPoint(nx, ny)) {
                                frontier_queue.push(Point(nx, ny));
                                visited[ny][nx] = true;
                            }
                        }
                    }
                }

                if (frontier.points.size() >= (size_t)min_frontier_size_) {
                    // Calculate centroid
                    int sum_x = 0, sum_y = 0;
                    for (const auto& point : frontier.points) {
                        sum_x += point.x;
                        sum_y += point.y;
                    }
                    frontier.centroid = Point(
                        sum_x / frontier.points.size(),
                        sum_y / frontier.points.size()
                    );
                    frontiers.push_back(frontier);
                }
            }

            // Add neighbors to queue
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    int nx = p.x + dx;
                    int ny = p.y + dy;
                    if (nx >= 0 && nx < (int)map_.info.width &&
                        ny >= 0 && ny < (int)map_.info.height &&
                        !visited[ny][nx] &&
                        map_.data[ny * map_.info.width + nx] != -1) {
                        queue.push(Point(nx, ny));
                        visited[ny][nx] = true;
                    }
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "Found %zu frontiers", frontiers.size());
        return frontiers;
    }

    Point findNearestFrontier(const std::vector<Frontier>& frontiers) {
        if (frontiers.empty()) return Point(0, 0);

        // Convert robot position to grid coordinates
        int robot_x = (current_pose_.pose.pose.position.x - map_.info.origin.position.x) 
                     / map_.info.resolution;
        int robot_y = (current_pose_.pose.pose.position.y - map_.info.origin.position.y) 
                     / map_.info.resolution;

        double min_dist = std::numeric_limits<double>::max();
        Point nearest(0, 0);

        for (const auto& frontier : frontiers) {
            double dx = frontier.centroid.x - robot_x;
            double dy = frontier.centroid.y - robot_y;
            double dist = std::sqrt(dx * dx + dy * dy);

            if (dist < min_dist) {
                min_dist = dist;
                nearest = frontier.centroid;
            }
        }

        return nearest;
    }

    void moveToPoint(const Point& target) {
        // Convert target point to world coordinates
        double target_x = target.x * map_.info.resolution + map_.info.origin.position.x;
        double target_y = target.y * map_.info.resolution + map_.info.origin.position.y;

        // Calculate direction to target
        double dx = target_x - current_pose_.pose.pose.position.x;
        double dy = target_y - current_pose_.pose.pose.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        // Get current yaw from quaternion
        double current_yaw = tf2::getYaw(current_pose_.pose.pose.orientation);
        
        // Calculate desired heading
        double desired_yaw = std::atan2(dy, dx);

        // Calculate angle difference
        double angle_diff = desired_yaw - current_yaw;
        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

        // Create velocity command
        geometry_msgs::msg::Twist cmd_vel;
        
        // Angular velocity control
        double angular_velocity = this->get_parameter("angular_velocity").as_double();
        cmd_vel.angular.z = std::clamp(
            angular_velocity * angle_diff,
            -angular_velocity,
            angular_velocity
        );

        // Linear velocity control
        if (std::abs(angle_diff) < 0.3) {
            double linear_velocity = this->get_parameter("linear_velocity").as_double();
            cmd_vel.linear.x = linear_velocity * std::min(1.0, distance);
        }

        cmd_vel_pub_->publish(cmd_vel);
    }

    void exploreLoop() {
        if (!map_received_ || !pose_received_ || !scan_received_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), 
                                *this->get_clock(), 
                                5000, // Warn every 5 seconds
                                "Waiting for data - Map: %d, Pose: %d, Scan: %d",
                                map_received_, pose_received_, scan_received_);
            return;
        }

        // Check exploration timeout
        double elapsed_time = (this->now() - start_time_).seconds();
        if (elapsed_time > this->get_parameter("exploration_timeout").as_double()) {
            if (current_state_ != FINISHED) {
                RCLCPP_INFO(this->get_logger(), "Exploration timeout reached. Stopping.");
                current_state_ = FINISHED;
                geometry_msgs::msg::Twist cmd_vel;
                cmd_vel_pub_->publish(cmd_vel);
            }
            return;
        }

        switch (current_state_) {
            case INITIALIZING:
                // Wait for data
                break;

            case SCANNING: {
                auto frontiers = detectFrontiers();
                if (!frontiers.empty()) {
                    target_frontier_ = findNearestFrontier(frontiers);
                    current_state_ = MOVING_TO_FRONTIER;
                    RCLCPP_INFO(this->get_logger(), "Moving to frontier at (%d, %d)", 
                        target_frontier_.x, target_frontier_.y);
                } else {
                    if (current_state_ != FINISHED) {
                        RCLCPP_INFO(this->get_logger(), "No more frontiers found. Exploration complete!");
                        current_state_ = FINISHED;
                        geometry_msgs::msg::Twist cmd_vel;
                        cmd_vel_pub_->publish(cmd_vel);
                    }
                }
                break;
            }

            case MOVING_TO_FRONTIER: {
                // Convert current position to grid coordinates
                int robot_x = (current_pose_.pose.pose.position.x - map_.info.origin.position.x) 
                             / map_.info.resolution;
                int robot_y = (current_pose_.pose.pose.position.y - map_.info.origin.position.y) 
                             / map_.info.resolution;

                // Check if we've reached the frontier
                double dist = std::sqrt(std::pow(robot_x - target_frontier_.x, 2) + 
                                      std::pow(robot_y - target_frontier_.y, 2));
                
                if (dist < this->get_parameter("goal_tolerance").as_double()) {
                    RCLCPP_INFO(this->get_logger(), "Reached frontier, starting rotation");
                    current_state_ = ROTATING;
                    rotation_start_time_ = this->now();
                    // Stop before starting rotation
                    geometry_msgs::msg::Twist cmd_vel;
                    cmd_vel_pub_->publish(cmd_vel);
                } else {
                    moveToPoint(target_frontier_);
                }
                break;
            }

            case ROTATING: {
                // Calculate remaining rotation time
                auto rotation_duration = this->now() - rotation_start_time_;
                double target_duration = this->get_parameter("rotation_duration").as_double();
                double remaining_time = target_duration - rotation_duration.seconds();
                
                if (remaining_time > 0) {
                    // Perform rotation with smooth velocity profile
                    geometry_msgs::msg::Twist cmd_vel;
                    double angular_velocity = this->get_parameter("angular_velocity").as_double();
                    
                    // Slow down near the end of rotation
                    if (remaining_time < 2.0) {
                        angular_velocity *= remaining_time / 2.0;
                    }
                    
                    cmd_vel.angular.z = angular_velocity;
                    cmd_vel_pub_->publish(cmd_vel);
                    
                    RCLCPP_DEBUG(this->get_logger(), "Rotating: %.1f seconds remaining", remaining_time);
                } else {
                    RCLCPP_INFO(this->get_logger(), "Rotation complete, returning to scanning");
                    current_state_ = SCANNING;
                    
                    // Stop rotation
                    geometry_msgs::msg::Twist stop_cmd;
                    cmd_vel_pub_->publish(stop_cmd);
                    
                    // Add a small delay before next scan
                    rclcpp::sleep_for(std::chrono::milliseconds(500));
                }
                break;
            }

            case FINISHED: {
                // Keep robot stopped
                geometry_msgs::msg::Twist cmd_vel;
                cmd_vel_pub_->publish(cmd_vel);
                break;
            }
        }
    }

private:
    // Publishers and Subscribers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    
    // TF2 related members
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr explore_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    // State variables
    nav_msgs::msg::OccupancyGrid map_;
    nav_msgs::msg::Odometry current_pose_;
    sensor_msgs::msg::LaserScan latest_scan_;
    
    bool map_received_ = false;
    bool pose_received_ = false;
    bool scan_received_ = false;
    ExplorationState current_state_;
    Point target_frontier_{0, 0};
    rclcpp::Time start_time_;
    rclcpp::Time rotation_start_time_;
    std::string robot_name_;
    int min_frontier_size_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrontierExplorer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
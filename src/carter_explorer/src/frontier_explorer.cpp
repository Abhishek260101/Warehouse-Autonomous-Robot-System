#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <queue>
#include <cmath>

class FrontierExplorer : public rclcpp::Node {
public:
    FrontierExplorer() : Node("frontier_explorer") {
        // Declare all parameters
        this->declare_parameter("robot_name", "carter1");
        this->declare_parameter("min_frontier_size", 10);
        this->declare_parameter("exploration_timeout", 300.0);
        this->declare_parameter("linear_velocity", 0.2);
        this->declare_parameter("angular_velocity", 0.3);
        this->declare_parameter("goal_tolerance", 0.5);
        this->declare_parameter("map_update_tolerance", 5.0);

        robot_name_ = this->get_parameter("robot_name").as_string();
        min_frontier_size_ = this->get_parameter("min_frontier_size").as_int();
        
        RCLCPP_INFO(this->get_logger(), "Starting Frontier Explorer for %s", robot_name_.c_str());

        // Publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/" + robot_name_ + "/cmd_vel", 10);

        // Subscribers with QoS settings
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", qos,
            std::bind(&FrontierExplorer::mapCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/" + robot_name_ + "/chassis/odom", qos,
            std::bind(&FrontierExplorer::odomCallback, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/" + robot_name_ + "/front_2d_lidar/scan", qos,
            std::bind(&FrontierExplorer::scanCallback, this, std::placeholders::_1));

        // Create timers
        explore_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&FrontierExplorer::exploreLoop, this));

        status_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&FrontierExplorer::printStatus, this));

        current_state_ = SCANNING;
        start_time_ = this->now();
        last_map_update_time_ = this->now();
    }


private:
    enum ExplorationState {
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

    void printStatus() {
        if (current_state_ == FINISHED) {
            return;
        }

        RCLCPP_INFO(this->get_logger(), 
            "Status - Map: %s, Odom: %s, Scan: %s, State: %d", 
            map_received_ ? "✓" : "✗",
            pose_received_ ? "✓" : "✗",
            scan_received_ ? "✓" : "✗",
            current_state_);

        if (map_received_) {
            int total_explored = 0;
            int total_unknown = 0;
            for (const auto& cell : map_.data) {
                if (cell >= 0) total_explored++;
                if (cell == -1) total_unknown++;
            }
            float explored_percentage = (float)total_explored / map_.data.size() * 100;
            RCLCPP_INFO(this->get_logger(), "Exploration progress: %.1f%%", explored_percentage);
        }
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        if (!map_received_) {
            RCLCPP_INFO(this->get_logger(), "Received first map update");
        }
        
        // Check if the map has actual data
        bool has_data = false;
        for (const auto& cell : msg->data) {
            if (cell != -1) {  // If any cell is not unknown
                has_data = true;
                break;
            }
        }
        
        if (has_data) {
            map_ = *msg;
            map_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Map updated - Size: %dx%d", 
                msg->info.width, msg->info.height);
        } else if (!map_received_) {
            // If we haven't received a valid map yet, start a small rotation
            // to help SLAM get initial readings
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.angular.z = 0.2;  // Slow rotation
            cmd_vel_pub_->publish(cmd_vel);
            RCLCPP_INFO_THROTTLE(this->get_logger(), 
                *this->get_clock(), 
                5000, // Log every 5 seconds
                "Waiting for valid map data, rotating slowly...");
        }
    }


    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_ = *msg;
        if (!pose_received_) {
            RCLCPP_INFO(this->get_logger(), "Received first odometry message");
        }
        pose_received_ = true;
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        latest_scan_ = *msg;
        if (!scan_received_) {
            RCLCPP_INFO(this->get_logger(), "Received first laser scan");
        }
        scan_received_ = true;
    }

    bool isFrontierPoint(int x, int y) {
        if (x < 0 || x >= (int)map_.info.width || 
            y < 0 || y >= (int)map_.info.height) {
            return false;
        }

        // Point must be unknown
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

        // Start flood fill from robot position
        std::queue<Point> queue;
        queue.push(Point(robot_x, robot_y));
        visited[robot_y][robot_x] = true;

        while (!queue.empty()) {
            Point p = queue.front();
            queue.pop();

            if (isFrontierPoint(p.x, p.y)) {
                // Start a new frontier
                Frontier frontier;
                std::queue<Point> frontier_queue;
                frontier_queue.push(p);
                visited[p.y][p.x] = true;

                // Collect all connected frontier points
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

                // Only add frontiers above minimum size
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

        // Get current yaw from quaternion using tf2::getYaw
        double current_yaw = tf2::getYaw(current_pose_.pose.pose.orientation);
        
        // Calculate desired heading
        double desired_yaw = std::atan2(dy, dx);

        // Calculate angle difference
        double angle_diff = desired_yaw - current_yaw;
        if (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        if (angle_diff < -M_PI) angle_diff += 2 * M_PI;

        // Create and publish velocity command
        geometry_msgs::msg::Twist cmd_vel;
        
        // Angular velocity control
        double angular_velocity = this->get_parameter("angular_velocity").as_double();
        cmd_vel.angular.z = angular_velocity * angle_diff;

        // Linear velocity control
        if (std::abs(angle_diff) < 0.3) {  // Only move forward when roughly facing the target
            double linear_velocity = this->get_parameter("linear_velocity").as_double();
            cmd_vel.linear.x = linear_velocity * std::min(1.0, distance);
        }

        cmd_vel_pub_->publish(cmd_vel);
    }

    void exploreLoop() {
        static rclcpp::Time last_valid_map_time = this->now();
        
        if (!map_received_ || !pose_received_ || !scan_received_) {
            if (!map_received_) {
                // If no map received for a while, trigger a rotation
                if ((this->now() - last_valid_map_time).seconds() > 
                    this->get_parameter("map_update_tolerance").as_double()) {
                    geometry_msgs::msg::Twist cmd_vel;
                    cmd_vel.angular.z = 0.2;
                    cmd_vel_pub_->publish(cmd_vel);
                }
            }
            RCLCPP_WARN_THROTTLE(this->get_logger(), 
                *this->get_clock(), 
                5000,
                "Waiting for data - Map: %d, Pose: %d, Scan: %d",
                map_received_, pose_received_, scan_received_);
            return;
        }
        
        last_valid_map_time = this->now();

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
                } else {// Continue moving to frontier
                    moveToPoint(target_frontier_);
                }
                break;
            }

            case ROTATING: {
                // Perform a 360-degree scan
                geometry_msgs::msg::Twist cmd_vel;
                cmd_vel.angular.z = this->get_parameter("angular_velocity").as_double();
                cmd_vel_pub_->publish(cmd_vel);

                // Check if we've completed the rotation (approximately 2π radians)
                auto rotation_duration = this->now() - rotation_start_time_;
                double angular_velocity = this->get_parameter("angular_velocity").as_double();
                double rotation_time = 2.0 * M_PI / angular_velocity;
                
                if (rotation_duration.seconds() > rotation_time) {
                    RCLCPP_INFO(this->get_logger(), "Rotation complete, returning to scanning");
                    current_state_ = SCANNING;
                    // Stop rotation
                    geometry_msgs::msg::Twist stop_cmd;
                    cmd_vel_pub_->publish(stop_cmd);
                }
                break;
            }

            case FINISHED: {
                // Exploration is complete, robot should be stopped
                geometry_msgs::msg::Twist cmd_vel;
                cmd_vel_pub_->publish(cmd_vel);
                break;
            }
        }
    }

    // Publishers and Subscribers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr explore_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::Time last_map_update_time_;


    // State variables
    nav_msgs::msg::OccupancyGrid map_;
    nav_msgs::msg::Odometry current_pose_;
    sensor_msgs::msg::LaserScan latest_scan_;
    bool map_received_ = false;
    bool pose_received_ = false;
    bool scan_received_ = false;
    ExplorationState current_state_;
    Point target_frontier_{0, 0};
    rclcpp::Time rotation_start_time_;
    rclcpp::Time start_time_;
    std::string robot_name_;
    int min_frontier_size_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrontierExplorer>());
    rclcpp::shutdown();
    return 0;
}
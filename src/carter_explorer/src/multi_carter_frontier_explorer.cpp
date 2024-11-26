#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <queue>
#include <vector>
#include <memory>

struct Point {
    int x, y;
    Point(int x_ = 0, int y_ = 0) : x(x_), y(y_) {}
};

struct Frontier {
    std::vector<Point> points;
    Point centroid;
    double size;
    bool being_explored;
};

class CarterFrontierExplorer : public rclcpp::Node {
public:
    explicit CarterFrontierExplorer(const std::string& robot_name)
    : Node("frontier_explorer_" + robot_name),
      robot_name_(robot_name),
      tf_buffer_(std::make_unique<tf2_ros::Buffer>(this->get_clock())),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
    {
        // Parameters
        this->declare_parameter("min_frontier_size", 10);
        this->declare_parameter("exploration_timeout", 300.0);
        this->declare_parameter("max_linear_speed", 0.5);
        this->declare_parameter("max_angular_speed", 1.0);
        
        // Publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/" + robot_name_ + "/cmd_vel", 10);
        
        // Subscribers
        // Subscribe to robot-specific map topic
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/" + robot_name_ + "/map", 10,  // Changed to use robot-specific map
            std::bind(&CarterFrontierExplorer::mapCallback, this, std::placeholders::_1));
            
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/" + robot_name_ + "/chassis/odom", 10,
            std::bind(&CarterFrontierExplorer::odomCallback, this, std::placeholders::_1));

        // Timer for exploration
        explore_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&CarterFrontierExplorer::exploreTimerCallback, this));

        RCLCPP_INFO(this->get_logger(), 
                   "Initialized frontier explorer for %s, using map: /%s/map", 
                   robot_name_.c_str(), 
                   robot_name_.c_str());
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        RCLCPP_INFO_ONCE(this->get_logger(), 
                        "[%s] Received first map update", 
                        robot_name_.c_str());
        current_map_ = msg;
        updateFrontiers();
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_ = msg->pose.pose;
        has_odom_ = true;
    }

    bool isFrontierCell(int x, int y) {
        if (!current_map_) return false;
        
        // Check if cell is out of bounds
        if (x < 0 || x >= (int)current_map_->info.width || 
            y < 0 || y >= (int)current_map_->info.height) {
            return false;
        }

        // Cell must be free
        if (getCellValue(x, y) != 0) return false;

        // Check 8-connected neighbors for unknown cells
        bool hasUnknownNeighbor = false;
        bool hasFreeNeighbor = false;

        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) continue;
                
                int nx = x + dx;
                int ny = y + dy;
                
                if (nx >= 0 && nx < (int)current_map_->info.width && 
                    ny >= 0 && ny < (int)current_map_->info.height) {
                    int value = getCellValue(nx, ny);
                    if (value == -1) hasUnknownNeighbor = true;
                    if (value == 0) hasFreeNeighbor = true;
                }
            }
        }

        return hasUnknownNeighbor && hasFreeNeighbor;
    }

    std::vector<Frontier> detectFrontiers() {
        std::vector<Frontier> frontiers;
        if (!current_map_) return frontiers;

        std::vector<std::vector<bool>> visited(
            current_map_->info.height,
            std::vector<bool>(current_map_->info.width, false));

        for (size_t y = 0; y < current_map_->info.height; y++) {
            for (size_t x = 0; x < current_map_->info.width; x++) {
                if (!visited[y][x] && isFrontierCell(x, y)) {
                    // Start flood fill
                    Frontier frontier;
                    std::queue<Point> queue;
                    Point start(x, y);
                    queue.push(start);
                    visited[y][x] = true;

                    while (!queue.empty()) {
                        Point p = queue.front();
                        queue.pop();
                        frontier.points.push_back(p);

                        // Check neighbors
                        for (int dx = -1; dx <= 1; dx++) {
                            for (int dy = -1; dy <= 1; dy++) {
                                int nx = p.x + dx;
                                int ny = p.y + dy;
                                
                                if (nx >= 0 && nx < (int)current_map_->info.width && 
                                    ny >= 0 && ny < (int)current_map_->info.height && 
                                    !visited[ny][nx] && 
                                    isFrontierCell(nx, ny)) {
                                    Point neighbor(nx, ny);
                                    queue.push(neighbor);
                                    visited[ny][nx] = true;
                                }
                            }
                        }
                    }

                    if (frontier.points.size() >= (size_t)this->get_parameter("min_frontier_size").as_int()) {
                        calculateFrontierCentroid(frontier);
                        frontier.being_explored = false;
                        frontiers.push_back(frontier);
                    }
                }
            }
        }

        return frontiers;
    }

    void calculateFrontierCentroid(Frontier& frontier) {
        int sum_x = 0, sum_y = 0;
        for (const auto& point : frontier.points) {
            sum_x += point.x;
            sum_y += point.y;
        }
        frontier.centroid.x = sum_x / frontier.points.size();
        frontier.centroid.y = sum_y / frontier.points.size();
        frontier.size = frontier.points.size();
    }

    geometry_msgs::msg::Point gridToWorld(const Point& grid_point) {
        geometry_msgs::msg::Point world_point;
        world_point.x = grid_point.x * current_map_->info.resolution + 
                       current_map_->info.origin.position.x;
        world_point.y = grid_point.y * current_map_->info.resolution + 
                       current_map_->info.origin.position.y;
        world_point.z = 0.0;
        return world_point;
    }

    double calculateDistance(const geometry_msgs::msg::Point& p1, 
                           const geometry_msgs::msg::Point& p2) {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    std::optional<Frontier> findBestFrontier() {
        if (frontiers_.empty() || !has_odom_) {
            return std::nullopt;
        }

        Frontier* best_frontier = nullptr;
        double best_score = std::numeric_limits<double>::infinity();
        geometry_msgs::msg::Point robot_pos = current_pose_.position;

        for (auto& frontier : frontiers_) {
            if (frontier.being_explored) continue;

            geometry_msgs::msg::Point frontier_pos = gridToWorld(frontier.centroid);
            double distance = calculateDistance(robot_pos, frontier_pos);
            
            // Score combines distance and frontier size
            double score = distance / frontier.size;
            
            if (score < best_score) {
                best_score = score;
                best_frontier = &frontier;
            }
        }

        if (best_frontier) {
            best_frontier->being_explored = true;
            return *best_frontier;
        }

        return std::nullopt;
    }

    void moveTowardsFrontier(const Frontier& frontier) {
        geometry_msgs::msg::Point target = gridToWorld(frontier.centroid);
        geometry_msgs::msg::Point robot_pos = current_pose_.position;

        // Calculate direction and distance
        double dx = target.x - robot_pos.x;
        double dy = target.y - robot_pos.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        // Calculate desired heading
        double desired_yaw = std::atan2(dy, dx);
        
        // Get current heading
        double roll, pitch, current_yaw;
        tf2::Quaternion q(
            current_pose_.orientation.x,
            current_pose_.orientation.y,
            current_pose_.orientation.z,
            current_pose_.orientation.w);
        tf2::Matrix3x3(q).getRPY(roll, pitch, current_yaw);

        // Calculate heading error
        double angle_diff = desired_yaw - current_yaw;
        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

        // Create velocity command
        geometry_msgs::msg::Twist cmd_vel;
        
        // Angular velocity (proportional control)
        double max_angular_speed = this->get_parameter("max_angular_speed").as_double();
        cmd_vel.angular.z = std::clamp(0.5 * angle_diff, -max_angular_speed, max_angular_speed);

        // Linear velocity (proportional control with angular threshold)
        if (std::abs(angle_diff) < 0.5) {  // Only move forward when roughly facing the target
            double max_linear_speed = this->get_parameter("max_linear_speed").as_double();
            cmd_vel.linear.x = std::clamp(0.3 * distance, 0.0, max_linear_speed);
        }

        cmd_vel_pub_->publish(cmd_vel);
    }

    void updateFrontiers() {
        frontiers_ = detectFrontiers();
        RCLCPP_INFO(this->get_logger(), 
                    "[%s] Found %zu frontiers", 
                    robot_name_.c_str(), 
                    frontiers_.size());
    }

    void exploreTimerCallback() {
        if (!current_map_ || !has_odom_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "[%s] Waiting for map or odometry data...",
                robot_name_.c_str());
            return;
        }

        auto best_frontier = findBestFrontier();
        if (!best_frontier) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "[%s] No valid frontiers found. Exploration might be complete.",
                robot_name_.c_str());
            // Stop the robot
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel_pub_->publish(cmd_vel);
            return;
        }

        moveTowardsFrontier(*best_frontier);
    }

    int getCellValue(int x, int y) {
        if (!current_map_) return -1;
        return current_map_->data[y * current_map_->info.width + x];
    }

    // Class members
    std::string robot_name_;
    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
    geometry_msgs::msg::Pose current_pose_;
    std::vector<Frontier> frontiers_;
    bool has_odom_ = false;

    // ROS components
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr explore_timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    
    // Create nodes for both robots
    auto carter1_node = std::make_shared<CarterFrontierExplorer>("carter1");
    auto carter2_node = std::make_shared<CarterFrontierExplorer>("carter2");
    
    executor->add_node(carter1_node);
    executor->add_node(carter2_node);
    
    executor->spin();
    
    rclcpp::shutdown();
    return 0;
}
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class SimpleExplorer : public rclcpp::Node {
public:
    SimpleExplorer() : Node("simple_explorer") {
        // Publisher for robot movement commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/carter1/cmd_vel", 10);

        // Subscribe to laser scan data
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/carter1/front_2d_lidar/scan", 10,
            std::bind(&SimpleExplorer::scan_callback, this, std::placeholders::_1));

        // Timer for movement updates
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SimpleExplorer::move_robot, this));
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        // Simple obstacle detection
        bool obstacle_detected = false;
        for (size_t i = 0; i < scan->ranges.size(); i++) {
            if (scan->ranges[i] < 1.0 && scan->ranges[i] > scan->range_min) {
                obstacle_detected = true;
                break;
            }
        }
        obstacle_ahead_ = obstacle_detected;
    }

    void move_robot() {
        auto cmd_vel = geometry_msgs::msg::Twist();
        
        if (obstacle_ahead_) {
            // Rotate in place when obstacle detected
            cmd_vel.angular.z = 0.5;
            cmd_vel.linear.x = 0.0;
        } else {
            // Move forward when path is clear
            cmd_vel.linear.x = 0.3;
            cmd_vel.angular.z = 0.0;
        }

        cmd_vel_pub_->publish(cmd_vel);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool obstacle_ahead_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleExplorer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

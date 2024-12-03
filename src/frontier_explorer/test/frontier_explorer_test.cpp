#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "frontier_explorer/frontier_explorer.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <memory>
#include <vector>

class TestFrontierExplorer : public ::testing::Test {
protected:
    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }

        rclcpp::NodeOptions options;
        options.automatically_declare_parameters_from_overrides(true);
        
        std::vector<rclcpp::Parameter> params = {
            rclcpp::Parameter("robot_namespace", "robot1"),
            rclcpp::Parameter("use_sim_time", false),
            rclcpp::Parameter("map_frame", "map")
        };
        options.parameter_overrides(params);

        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        node_ = std::make_shared<FrontierExplorer>(options);
        executor_->add_node(node_);

        // Setup default scan message for testing
        default_scan_ = std::make_shared<sensor_msgs::msg::LaserScan>();
        default_scan_->header.frame_id = "robot1/front_3d_lidar";
        default_scan_->angle_min = -M_PI/4;
        default_scan_->angle_max = M_PI/4;
        default_scan_->angle_increment = M_PI/180;  // 1 degree
        default_scan_->range_min = 0.1;
        default_scan_->range_max = 10.0;
        default_scan_->ranges.resize(90);
        std::fill(default_scan_->ranges.begin(), default_scan_->ranges.end(), 5.0f);
    }

    void TearDown() override {
        executor_->remove_node(node_);
        node_.reset();
        executor_.reset();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    void test_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        node_->scan_callback(msg);
    }

    std::string get_robot_namespace() { return node_->robot_namespace_; }
    std::string get_map_frame() { return node_->map_frame_; }
    sensor_msgs::msg::LaserScan::SharedPtr get_current_scan() { return node_->current_scan_; }

    std::shared_ptr<FrontierExplorer> node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    sensor_msgs::msg::LaserScan::SharedPtr default_scan_;

    void spin_some(const std::chrono::nanoseconds timeout = std::chrono::milliseconds(100)) {
        executor_->spin_some(timeout);
    }
};

// Basic node tests
TEST_F(TestFrontierExplorer, TestNodeCreation) {
    ASSERT_NE(node_, nullptr);
    std::string node_name = std::string(node_->get_name());
    EXPECT_EQ(node_name, "frontier_explorer");
    EXPECT_EQ(get_robot_namespace(), "robot1");
}

TEST_F(TestFrontierExplorer, TestParameterInitialization) {
    bool use_sim_time;
    EXPECT_NO_THROW({
        use_sim_time = node_->get_parameter("use_sim_time").as_bool();
    });
    EXPECT_FALSE(use_sim_time);
    EXPECT_EQ(get_map_frame(), "map");
}

// Scan data tests
TEST_F(TestFrontierExplorer, TestNormalScanHandling) {
    EXPECT_NO_THROW({
        test_scan_callback(default_scan_);
        spin_some();
    });
    
    auto current_scan = get_current_scan();
    ASSERT_NE(current_scan, nullptr);
    EXPECT_EQ(current_scan->ranges.size(), 90);
    EXPECT_EQ(current_scan->header.frame_id, "robot1/front_3d_lidar");
}

TEST_F(TestFrontierExplorer, TestInvalidScanData) {
    // Test empty scan
    auto empty_scan = std::make_shared<sensor_msgs::msg::LaserScan>();
    EXPECT_NO_THROW({
        test_scan_callback(empty_scan);
        spin_some();
    });

    // Test scan with invalid ranges
    auto invalid_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*default_scan_);
    std::fill(invalid_scan->ranges.begin(), invalid_scan->ranges.end(), 
              std::numeric_limits<float>::infinity());
    EXPECT_NO_THROW({
        test_scan_callback(invalid_scan);
        spin_some();
    });

    // Test scan with ranges below minimum
    auto below_min_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*default_scan_);
    std::fill(below_min_scan->ranges.begin(), below_min_scan->ranges.end(), 0.05f);
    EXPECT_NO_THROW({
        test_scan_callback(below_min_scan);
        spin_some();
    });
}

TEST_F(TestFrontierExplorer, TestObstacleDetection) {
    // Create scan with obstacle in front
    auto obstacle_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*default_scan_);
    obstacle_scan->ranges[45] = 0.5f;  // Obstacle at center

    EXPECT_NO_THROW({
        test_scan_callback(obstacle_scan);
        spin_some();
    });
}

TEST_F(TestFrontierExplorer, TestFrameIDHandling) {
    // Test with different frame IDs
    auto custom_frame_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*default_scan_);
    custom_frame_scan->header.frame_id = "custom_frame";
    
    EXPECT_NO_THROW({
        test_scan_callback(custom_frame_scan);
        spin_some();
    });
    
    auto current_scan = get_current_scan();
    ASSERT_NE(current_scan, nullptr);
    EXPECT_EQ(current_scan->header.frame_id, "custom_frame");
}

TEST_F(TestFrontierExplorer, TestScanRangeValidation) {
    // Test different range values
    auto range_test_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*default_scan_);
    range_test_scan->ranges[0] = 0.0f;                                    // Below min
    range_test_scan->ranges[1] = range_test_scan->range_min;             // At min
    range_test_scan->ranges[2] = range_test_scan->range_max;             // At max
    range_test_scan->ranges[3] = range_test_scan->range_max + 1.0f;      // Above max
    range_test_scan->ranges[4] = std::numeric_limits<float>::infinity(); // Infinity
    range_test_scan->ranges[5] = std::numeric_limits<float>::quiet_NaN();// NaN

    EXPECT_NO_THROW({
        test_scan_callback(range_test_scan);
        spin_some();
    });
}

TEST_F(TestFrontierExplorer, TestEdgeCases) {
    // Test scan with extreme angles
    auto extreme_angles_scan = std::make_shared<sensor_msgs::msg::LaserScan>();
    extreme_angles_scan->header.frame_id = "robot1/front_3d_lidar";
    extreme_angles_scan->angle_min = -M_PI;
    extreme_angles_scan->angle_max = M_PI;
    extreme_angles_scan->angle_increment = M_PI/180;
    extreme_angles_scan->range_min = 0.1;
    extreme_angles_scan->range_max = 10.0;
    extreme_angles_scan->ranges.resize(360);
    std::fill(extreme_angles_scan->ranges.begin(), extreme_angles_scan->ranges.end(), 5.0f);

    EXPECT_NO_THROW({
        test_scan_callback(extreme_angles_scan);
        spin_some();
    });
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
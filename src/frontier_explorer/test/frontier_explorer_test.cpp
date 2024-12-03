#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "frontier_explorer/frontier_explorer.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <memory>
#include <vector>

class TestFrontierExplorer : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);

        rclcpp::NodeOptions options;
        options.automatically_declare_parameters_from_overrides(true);
        
        std::vector<rclcpp::Parameter> params = {
            rclcpp::Parameter("robot_namespace", "robot1"),
            rclcpp::Parameter("use_sim_time", false),
            rclcpp::Parameter("map_frame", "map")
        };
        options.parameter_overrides(params);

        node_ = std::make_shared<FrontierExplorer>(options);
        setupDefaultScan();
        setupTransforms();
    }

    void TearDown() override {
        node_.reset();
        rclcpp::shutdown();
    }

    void setupDefaultScan() {
        default_scan_ = std::make_shared<sensor_msgs::msg::LaserScan>();
        default_scan_->header.frame_id = "robot1/front_3d_lidar";
        default_scan_->angle_min = -M_PI/4;
        default_scan_->angle_max = M_PI/4;
        default_scan_->angle_increment = M_PI/180;  // 1 degree
        default_scan_->range_min = 0.1;
        default_scan_->range_max = 10.0;
        default_scan_->ranges.resize(90);
        std::fill(default_scan_->ranges.begin(), default_scan_->ranges.end(), 5.0f);
        default_scan_->header.stamp = node_->now();
    }

    void setupTransforms() {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = node_->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "base_link";
        transform.transform.rotation.w = 1.0;
        node_->get_tf_buffer()->setTransform(transform, "test_authority", true);
    }

    void spin_some() {
        rclcpp::spin_some(node_);
    }

    std::shared_ptr<FrontierExplorer> node_;
    sensor_msgs::msg::LaserScan::SharedPtr default_scan_;
};

TEST_F(TestFrontierExplorer, TestNodeInitialization) {
    EXPECT_EQ(std::string(node_->get_name()), "frontier_explorer");
    EXPECT_EQ(node_->get_robot_namespace(), "robot1");
    EXPECT_EQ(node_->get_map_frame(), "map");
}

TEST_F(TestFrontierExplorer, TestScanProcessing) {
    node_->scan_callback(default_scan_);
    spin_some();
    auto current_scan = node_->get_current_scan();
    ASSERT_NE(current_scan, nullptr);
    EXPECT_EQ(current_scan->ranges.size(), 90);
    EXPECT_EQ(current_scan->header.frame_id, "robot1/front_3d_lidar");
}

TEST_F(TestFrontierExplorer, TestObstacleDetection) {
    auto obstacle_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*default_scan_);
    obstacle_scan->ranges[45] = 0.5f;  // Add obstacle in center
    
    EXPECT_NO_THROW({
        node_->scan_callback(obstacle_scan);
        node_->exploration_callback();
        spin_some();
    });
}

TEST_F(TestFrontierExplorer, TestMultipleObstacles) {
    auto scan = std::make_shared<sensor_msgs::msg::LaserScan>(*default_scan_);
    scan->ranges[30] = 0.8f;  // Left
    scan->ranges[45] = 0.5f;  // Center
    scan->ranges[60] = 0.7f;  // Right
    
    EXPECT_NO_THROW({
        node_->scan_callback(scan);
        node_->exploration_callback();
        spin_some();
    });
}

TEST_F(TestFrontierExplorer, TestInvalidScans) {
    // Empty scan
    auto empty_scan = std::make_shared<sensor_msgs::msg::LaserScan>();
    EXPECT_NO_THROW({
        node_->scan_callback(empty_scan);
        node_->exploration_callback();
        spin_some();
    });

    // Invalid ranges
    auto invalid_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*default_scan_);
    std::fill(invalid_scan->ranges.begin(), invalid_scan->ranges.end(), 
              std::numeric_limits<float>::infinity());
    EXPECT_NO_THROW({
        node_->scan_callback(invalid_scan);
        node_->exploration_callback();
        spin_some();
    });
}

TEST_F(TestFrontierExplorer, TestRangeValidation) {
    auto scan = std::make_shared<sensor_msgs::msg::LaserScan>(*default_scan_);
    
    // Below minimum range
    std::fill(scan->ranges.begin(), scan->ranges.end(), 0.05f);
    EXPECT_NO_THROW({
        node_->scan_callback(scan);
        node_->exploration_callback();
        spin_some();
    });

    // Above maximum range
    std::fill(scan->ranges.begin(), scan->ranges.end(), 15.0f);
    EXPECT_NO_THROW({
        node_->scan_callback(scan);
        node_->exploration_callback();
        spin_some();
    });
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
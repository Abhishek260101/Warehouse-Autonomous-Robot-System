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
        
        // Set up test parameters
        std::vector<rclcpp::Parameter> params = {
            rclcpp::Parameter("robot_namespace", "robot1"),
            rclcpp::Parameter("use_sim_time", false),
            rclcpp::Parameter("map_frame", "map")
        };
        
        rclcpp::NodeOptions options;
        options.automatically_declare_parameters_from_overrides(true);
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

    std::shared_ptr<FrontierExplorer> node_;
    sensor_msgs::msg::LaserScan::SharedPtr default_scan_;
};

// Basic Initialization Tests
TEST_F(TestFrontierExplorer, TestNodeInitialization) {
    EXPECT_EQ(std::string(node_->get_name()), "frontier_explorer");
    EXPECT_EQ(node_->get_robot_namespace(), "robot1");
    EXPECT_EQ(node_->get_map_frame(), "map");
    EXPECT_NE(node_->get_tf_buffer(), nullptr);
}

// Scan Callback Tests
TEST_F(TestFrontierExplorer, TestScanCallbackWithValidData) {
    node_->scan_callback(default_scan_);
    auto current_scan = node_->get_current_scan();
    
    ASSERT_NE(current_scan, nullptr);
    EXPECT_EQ(current_scan->ranges.size(), 90);
    EXPECT_EQ(current_scan->header.frame_id, "robot1/front_3d_lidar");
    
    const double epsilon = 1e-6;
    EXPECT_NEAR(current_scan->angle_min, -M_PI/4, epsilon);
    EXPECT_NEAR(current_scan->angle_max, M_PI/4, epsilon);
}

TEST_F(TestFrontierExplorer, TestScanCallbackWithEmptyFrameId) {
    auto scan = std::make_shared<sensor_msgs::msg::LaserScan>(*default_scan_);
    scan->header.frame_id = "";
    
    node_->scan_callback(scan);
    auto current_scan = node_->get_current_scan();
    
    ASSERT_NE(current_scan, nullptr);
    EXPECT_EQ(current_scan->header.frame_id, "robot1/front_3d_lidar");
}

// Frontier Detection Tests
TEST_F(TestFrontierExplorer, TestFindFrontiersWithNullScan) {
    sensor_msgs::msg::LaserScan::SharedPtr null_scan;
    EXPECT_NO_THROW(node_->find_frontiers(null_scan));
}

TEST_F(TestFrontierExplorer, TestFindFrontiersWithClearPath) {
    // All ranges set to 5.0m (clear path)
    node_->scan_callback(default_scan_);
    EXPECT_NO_THROW(node_->find_frontiers(default_scan_));
}

TEST_F(TestFrontierExplorer, TestFindFrontiersWithObstacle) {
    auto obstacle_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*default_scan_);
    // Add obstacle at 1m in front
    obstacle_scan->ranges[45] = 1.0f;
    
    node_->scan_callback(obstacle_scan);
    EXPECT_NO_THROW(node_->find_frontiers(obstacle_scan));
}

TEST_F(TestFrontierExplorer, TestFindFrontiersWithAllInvalidReadings) {
    auto invalid_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*default_scan_);
    std::fill(invalid_scan->ranges.begin(), invalid_scan->ranges.end(), 
              std::numeric_limits<float>::infinity());
    
    node_->scan_callback(invalid_scan);
    EXPECT_NO_THROW(node_->find_frontiers(invalid_scan));
}

TEST_F(TestFrontierExplorer, TestFindFrontiersWithMixedReadings) {
    auto mixed_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*default_scan_);
    // Set various types of readings
    mixed_scan->ranges[0] = std::numeric_limits<float>::infinity();  // Invalid
    mixed_scan->ranges[1] = 0.05f;  // Below min range
    mixed_scan->ranges[2] = 15.0f;  // Above max range
    mixed_scan->ranges[3] = 1.1f;   // Valid, near obstacle
    mixed_scan->ranges[4] = 5.0f;   // Valid, clear path
    
    node_->scan_callback(mixed_scan);
    EXPECT_NO_THROW(node_->find_frontiers(mixed_scan));
}

// Exploration Callback Tests
TEST_F(TestFrontierExplorer, TestExplorationCallbackWithoutScan) {
    EXPECT_NO_THROW(node_->exploration_callback());
}

TEST_F(TestFrontierExplorer, TestExplorationCallbackWithScan) {
    node_->scan_callback(default_scan_);
    EXPECT_NO_THROW(node_->exploration_callback());
}

// Transform Tests
TEST_F(TestFrontierExplorer, TestTransformAvailability) {
    EXPECT_NE(node_->get_tf_buffer(), nullptr);
    EXPECT_TRUE(node_->get_tf_buffer()->canTransform("map", "base_link", tf2::TimePointZero));
}

// Edge Cases and Error Handling
TEST_F(TestFrontierExplorer, TestScanWithZeroRanges) {
    auto zero_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*default_scan_);
    zero_scan->ranges.clear();
    
    EXPECT_NO_THROW({
        node_->scan_callback(zero_scan);
        node_->exploration_callback();
    });
}

TEST_F(TestFrontierExplorer, TestScanWithAllMinRanges) {
    auto min_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*default_scan_);
    std::fill(min_scan->ranges.begin(), min_scan->ranges.end(), min_scan->range_min);
    
    EXPECT_NO_THROW({
        node_->scan_callback(min_scan);
        node_->find_frontiers(min_scan);
    });
}

TEST_F(TestFrontierExplorer, TestScanWithAllMaxRanges) {
    auto max_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*default_scan_);
    std::fill(max_scan->ranges.begin(), max_scan->ranges.end(), max_scan->range_max);
    
    EXPECT_NO_THROW({
        node_->scan_callback(max_scan);
        node_->find_frontiers(max_scan);
    });
}

// Test turning direction logic
TEST_F(TestFrontierExplorer, TestTurningDirectionSelection) {
    auto scan = std::make_shared<sensor_msgs::msg::LaserScan>(*default_scan_);
    
    // Create obstacle pattern that should trigger turning
    for (size_t i = 40; i < 50; i++) {
        scan->ranges[i] = 1.0;  // Obstacle within 1m
    }
    
    // Setup transforms properly
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = node_->now();
    transform.header.frame_id = "odom";  // Add odom frame
    transform.child_frame_id = "base_link";
    transform.transform.rotation.w = 1.0;
    node_->get_tf_buffer()->setTransform(transform, "test_authority", true);
    
    // Test multiple cycles to check turning behavior
    for (int i = 0; i < 3; i++) {
        node_->scan_callback(scan);
        node_->find_frontiers(scan);
        rclcpp::spin_some(node_);
    }
}

// Test clear path scenario with random turning
TEST_F(TestFrontierExplorer, TestClearPathBehavior) {
    auto scan = std::make_shared<sensor_msgs::msg::LaserScan>(*default_scan_);
    
    // Set all ranges to indicate clear path
    std::fill(scan->ranges.begin(), scan->ranges.end(), 5.0f);
    
    // Setup proper transform chain
    geometry_msgs::msg::TransformStamped odom_transform;
    odom_transform.header.stamp = node_->now();
    odom_transform.header.frame_id = "map";
    odom_transform.child_frame_id = "odom";
    odom_transform.transform.rotation.w = 1.0;
    node_->get_tf_buffer()->setTransform(odom_transform, "test_authority", true);
    
    geometry_msgs::msg::TransformStamped base_transform;
    base_transform.header.stamp = node_->now();
    base_transform.header.frame_id = "odom";
    base_transform.child_frame_id = "base_link";
    base_transform.transform.rotation.w = 1.0;
    node_->get_tf_buffer()->setTransform(base_transform, "test_authority", true);
    
    node_->scan_callback(scan);
    node_->find_frontiers(scan);
}

// Test obstacle avoidance with multiple obstacles
TEST_F(TestFrontierExplorer, TestMultipleObstacleAvoidance) {
    auto scan = std::make_shared<sensor_msgs::msg::LaserScan>(*default_scan_);
    
    // Create complex obstacle pattern
    scan->ranges[30] = 1.0;  // Left obstacle
    scan->ranges[45] = 1.1;  // Front obstacle
    scan->ranges[60] = 1.0;  // Right obstacle
    
    // Setup transform chain
    geometry_msgs::msg::TransformStamped odom_transform;
    odom_transform.header.stamp = node_->now();
    odom_transform.header.frame_id = "map";
    odom_transform.child_frame_id = "odom";
    odom_transform.transform.rotation.w = 1.0;
    node_->get_tf_buffer()->setTransform(odom_transform, "test_authority", true);
    
    geometry_msgs::msg::TransformStamped base_transform;
    base_transform.header.stamp = node_->now();
    base_transform.header.frame_id = "odom";
    base_transform.child_frame_id = "base_link";
    base_transform.transform.rotation.w = 1.0;
    node_->get_tf_buffer()->setTransform(base_transform, "test_authority", true);
    
    node_->scan_callback(scan);
    node_->find_frontiers(scan);
}

// Test best direction finding logic
TEST_F(TestFrontierExplorer, TestBestDirectionSelection) {
    auto scan = std::make_shared<sensor_msgs::msg::LaserScan>(*default_scan_);
    
    // Create a pattern where right side is clearer than left
    for (size_t i = 0; i < scan->ranges.size()/2; i++) {
        scan->ranges[i] = 1.0;  // Left side obstacles
    }
    for (size_t i = scan->ranges.size()/2; i < scan->ranges.size(); i++) {
        scan->ranges[i] = 5.0;  // Right side clear
    }
    
    // Setup transforms
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = node_->now();
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    transform.transform.rotation.w = 1.0;
    node_->get_tf_buffer()->setTransform(transform, "test_authority", true);
    
    node_->scan_callback(scan);
    node_->find_frontiers(scan);
}

// Test valid readings count logic
TEST_F(TestFrontierExplorer, TestValidReadingsCount) {
    auto scan = std::make_shared<sensor_msgs::msg::LaserScan>(*default_scan_);
    
    // Create scan with exactly 4 valid readings in front arc
    std::fill(scan->ranges.begin(), scan->ranges.end(), 
              std::numeric_limits<float>::infinity());
    
    size_t front_start_idx = (size_t)(((-M_PI/6) - scan->angle_min) / scan->angle_increment);
    for (size_t i = front_start_idx; i < front_start_idx + 4; i++) {
        scan->ranges[i] = 2.0;
    }
    
    // Setup transform
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = node_->now();
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    transform.transform.rotation.w = 1.0;
    node_->get_tf_buffer()->setTransform(transform, "test_authority", true);
    
    node_->scan_callback(scan);
    node_->find_frontiers(scan);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
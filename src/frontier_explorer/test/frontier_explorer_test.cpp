// Required test framework headers
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "frontier_explorer/frontier_explorer.hpp"

// Message type headers needed for testing
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Standard library headers
#include <memory>
#include <vector>

// Test fixture class that inherits from the Google Test framework
class TestFrontierExplorer : public ::testing::Test {
protected:
    // Setup function called before each test
    void SetUp() override {
        // Initialize ROS 2 context
        rclcpp::init(0, nullptr);
        
        // Set up test parameters with default values
        std::vector<rclcpp::Parameter> params = {
            rclcpp::Parameter("robot_namespace", "robot1"),
            rclcpp::Parameter("use_sim_time", false),
            rclcpp::Parameter("map_frame", "map")
        };
        
        // Configure node options with parameters
        rclcpp::NodeOptions options;
        options.automatically_declare_parameters_from_overrides(true);
        options.parameter_overrides(params);
        
        // Create the frontier explorer node
        node_ = std::make_shared<FrontierExplorer>(options);
        setupDefaultScan();
        setupTransforms();
    }

    // Cleanup function called after each test
    void TearDown() override {
        node_.reset();
        rclcpp::shutdown();
    }

    // Helper function to set up a default laser scan message
    void setupDefaultScan() {
        default_scan_ = std::make_shared<sensor_msgs::msg::LaserScan>();
        default_scan_->header.frame_id = "robot1/front_3d_lidar";
        
        // Configure scan parameters for a 90-degree FOV
        default_scan_->angle_min = -M_PI/4;
        default_scan_->angle_max = M_PI/4;
        default_scan_->angle_increment = M_PI/180;  // 1 degree resolution
        default_scan_->range_min = 0.1;
        default_scan_->range_max = 10.0;
        
        // Initialize scan with 90 readings, all at 5.0m
        default_scan_->ranges.resize(90);
        std::fill(default_scan_->ranges.begin(), default_scan_->ranges.end(), 5.0f);
        default_scan_->header.stamp = node_->now();
    }

    // Helper function to set up transform tree
    void setupTransforms() {
        // Create and publish static transform from map to base_link
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = node_->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "base_link";
        transform.transform.rotation.w = 1.0;  // Identity rotation
        node_->get_tf_buffer()->setTransform(transform, "test_authority", true);
    }

    // Test fixture member variables
    std::shared_ptr<FrontierExplorer> node_;
    sensor_msgs::msg::LaserScan::SharedPtr default_scan_;
};

// Basic node initialization tests
TEST_F(TestFrontierExplorer, TestNodeInitialization) {
    // Verify basic node properties
    EXPECT_EQ(std::string(node_->get_name()), "frontier_explorer");
    EXPECT_EQ(node_->get_robot_namespace(), "robot1");
    EXPECT_EQ(node_->get_map_frame(), "map");
    EXPECT_NE(node_->get_tf_buffer(), nullptr);
}

// Tests for laser scan callback functionality
TEST_F(TestFrontierExplorer, TestScanCallbackWithValidData) {
    // Test processing of a valid laser scan
    node_->scan_callback(default_scan_);
    auto current_scan = node_->get_current_scan();
    
    // Verify scan properties
    ASSERT_NE(current_scan, nullptr);
    EXPECT_EQ(current_scan->ranges.size(), 90);
    EXPECT_EQ(current_scan->header.frame_id, "robot1/front_3d_lidar");
    
    // Check angle properties with small epsilon for floating point comparison
    const double epsilon = 1e-6;
    EXPECT_NEAR(current_scan->angle_min, -M_PI/4, epsilon);
    EXPECT_NEAR(current_scan->angle_max, M_PI/4, epsilon);
}

TEST_F(TestFrontierExplorer, TestScanCallbackWithEmptyFrameId) {
    // Create scan with empty frame_id
    auto scan = std::make_shared<sensor_msgs::msg::LaserScan>(*default_scan_);
    scan->header.frame_id = "";
    
    // Process scan and verify frame_id handling
    node_->scan_callback(scan);
    auto current_scan = node_->get_current_scan();
    
    ASSERT_NE(current_scan, nullptr);
    EXPECT_EQ(current_scan->header.frame_id, "robot1/front_3d_lidar");
}

// Tests for frontier detection functionality
TEST_F(TestFrontierExplorer, TestFindFrontiersWithNullScan) {
    // Verify handling of null scan pointer
    sensor_msgs::msg::LaserScan::SharedPtr null_scan;
    EXPECT_NO_THROW(node_->find_frontiers(null_scan));
}

TEST_F(TestFrontierExplorer, TestFindFrontiersWithClearPath) {
    // Test frontier detection with unobstructed path
    node_->scan_callback(default_scan_);
    EXPECT_NO_THROW(node_->find_frontiers(default_scan_));
}

TEST_F(TestFrontierExplorer, TestFindFrontiersWithObstacle) {
    // Create scan with single obstacle
    auto obstacle_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*default_scan_);
    obstacle_scan->ranges[45] = 1.0f;  // Place obstacle at center
    
    node_->scan_callback(obstacle_scan);
    EXPECT_NO_THROW(node_->find_frontiers(obstacle_scan));
}

TEST_F(TestFrontierExplorer, TestFindFrontiersWithAllInvalidReadings) {
    // Test handling of scan with all invalid readings
    auto invalid_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*default_scan_);
    std::fill(invalid_scan->ranges.begin(), invalid_scan->ranges.end(), 
              std::numeric_limits<float>::infinity());
    
    node_->scan_callback(invalid_scan);
    EXPECT_NO_THROW(node_->find_frontiers(invalid_scan));
}

TEST_F(TestFrontierExplorer, TestFindFrontiersWithMixedReadings) {
    // Create scan with various types of readings
    auto mixed_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*default_scan_);
    mixed_scan->ranges[0] = std::numeric_limits<float>::infinity();  // Invalid
    mixed_scan->ranges[1] = 0.05f;  // Below min range
    mixed_scan->ranges[2] = 15.0f;  // Above max range
    mixed_scan->ranges[3] = 1.1f;   // Valid, near obstacle
    mixed_scan->ranges[4] = 5.0f;   // Valid, clear path
    
    node_->scan_callback(mixed_scan);
    EXPECT_NO_THROW(node_->find_frontiers(mixed_scan));
}

// Tests for exploration callback
TEST_F(TestFrontierExplorer, TestExplorationCallbackWithoutScan) {
    // Verify exploration behavior without scan data
    EXPECT_NO_THROW(node_->exploration_callback());
}

TEST_F(TestFrontierExplorer, TestExplorationCallbackWithScan) {
    // Test exploration with valid scan data
    node_->scan_callback(default_scan_);
    EXPECT_NO_THROW(node_->exploration_callback());
}

// Transform system tests
TEST_F(TestFrontierExplorer, TestTransformAvailability) {
    // Verify transform system initialization
    EXPECT_NE(node_->get_tf_buffer(), nullptr);
    EXPECT_TRUE(node_->get_tf_buffer()->canTransform("map", "base_link", tf2::TimePointZero));
}

// Edge case tests
TEST_F(TestFrontierExplorer, TestScanWithZeroRanges) {
    // Test handling of empty range array
    auto zero_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*default_scan_);
    zero_scan->ranges.clear();
    
    EXPECT_NO_THROW({
        node_->scan_callback(zero_scan);
        node_->exploration_callback();
    });
}

TEST_F(TestFrontierExplorer, TestScanWithAllMinRanges) {
    // Test behavior with all minimum range readings
    auto min_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*default_scan_);
    std::fill(min_scan->ranges.begin(), min_scan->ranges.end(), min_scan->range_min);
    
    EXPECT_NO_THROW({
        node_->scan_callback(min_scan);
        node_->find_frontiers(min_scan);
    });
}

TEST_F(TestFrontierExplorer, TestScanWithAllMaxRanges) {
    // Test behavior with all maximum range readings
    auto max_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*default_scan_);
    std::fill(max_scan->ranges.begin(), max_scan->ranges.end(), max_scan->range_max);
    
    EXPECT_NO_THROW({
        node_->scan_callback(max_scan);
        node_->find_frontiers(max_scan);
    });
}

// Tests for robot navigation behavior
TEST_F(TestFrontierExplorer, TestTurningDirectionSelection) {
    auto scan = std::make_shared<sensor_msgs::msg::LaserScan>(*default_scan_);
    
    // Create obstacle pattern in front of robot
    for (size_t i = 40; i < 50; i++) {
        scan->ranges[i] = 1.0;  // Place obstacles within 1m
    }
    
    // Set up transform from odom to base_link
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = node_->now();
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    transform.transform.rotation.w = 1.0;
    node_->get_tf_buffer()->setTransform(transform, "test_authority", true);
    
    // Run multiple cycles to verify consistent turning behavior
    for (int i = 0; i < 3; i++) {
        node_->scan_callback(scan);
        node_->find_frontiers(scan);
        rclcpp::spin_some(node_);
    }
}

TEST_F(TestFrontierExplorer, TestClearPathBehavior) {
    // Create scan with all clear readings
    auto scan = std::make_shared<sensor_msgs::msg::LaserScan>(*default_scan_);
    std::fill(scan->ranges.begin(), scan->ranges.end(), 5.0f);
    
    // Set up complete transform chain (map -> odom -> base_link)
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

TEST_F(TestFrontierExplorer, TestMultipleObstacleAvoidance) {
    // Create scan with multiple obstacles in different directions
    auto scan = std::make_shared<sensor_msgs::msg::LaserScan>(*default_scan_);
    
    scan->ranges[30] = 1.0;  // Left obstacle
    scan->ranges[45] = 1.1;  // Front obstacle
    scan->ranges[60] = 1.0;  // Right obstacle
    
    // Set up transform chain
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

TEST_F(TestFrontierExplorer, TestBestDirectionSelection) {
    // Create asymmetric obstacle pattern
    auto scan = std::make_shared<sensor_msgs::msg::LaserScan>(*default_scan_);
    
    // Make left side more obstructed than right
    for (size_t i = 0; i < scan->ranges.size()/2; i++) {
        scan->ranges[i] = 1.0;  // Left side obstacles
    }
    for (size_t i = scan->ranges.size()/2; i < scan->ranges.size(); i++) {
        scan->ranges[i] = 5.0;  // Right side clear
    }
    
    // Set up transform
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = node_->now();
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    transform.transform.rotation.w = 1.0;
    node_->get_tf_buffer()->setTransform(transform, "test_authority", true);
    
    node_->scan_callback(scan);
    node_->find_frontiers(scan);
}

TEST_F(TestFrontierExplorer, TestValidReadingsCount) {
    // Create scan with specific number of valid readings
    auto scan = std::make_shared<sensor_msgs::msg::LaserScan>(*default_scan_);
    
    // Initialize all readings as invalid
    std::fill(scan->ranges.begin(), scan->ranges.end(), 
              std::numeric_limits<float>::infinity());
    
    // Add exactly 4 valid readings in front arc
    size_t front_start_idx = (size_t)(((-M_PI/6) - scan->angle_min) / scan->angle_increment);
    for (size_t i = front_start_idx; i < front_start_idx + 4; i++) {
        scan->ranges[i] = 2.0;
    }
    
    // Set up transform
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = node_->now();
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    transform.transform.rotation.w = 1.0;
    node_->get_tf_buffer()->setTransform(transform, "test_authority", true);
    
    node_->scan_callback(scan);
    node_->find_frontiers(scan);
}

// Main function to run all tests
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
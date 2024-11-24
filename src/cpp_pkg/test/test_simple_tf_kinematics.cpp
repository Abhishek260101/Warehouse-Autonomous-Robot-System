#include <gtest/gtest.h>
#include <memory>
#include "simple_tf_kinematics.hpp"

class TestSimpleTfKinematics : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<SimpleTfKinematics>("test_simple_tf_kinematics");
    }

    void TearDown() override {
        node_.reset();
        rclcpp::shutdown();
    }

    std::shared_ptr<SimpleTfKinematics> node_;
};

TEST_F(TestSimpleTfKinematics, TestStaticTransform) {
    // Allow time for static transform to be published
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    // Create service client
    auto client = node_->create_client<robot_interface::srv::GetTransform>("get_transform");
    
    auto request = std::make_shared<robot_interface::srv::GetTransform::Request>();
    request->frame_id = "robot_base";
    request->child_frame_id = "robot_top";
    
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    auto result = client->async_send_request(request);
    
    // Wait for the result
    ASSERT_EQ(rclcpp::spin_until_future_complete(node_, result), 
              rclcpp::FutureReturnCode::SUCCESS);
              
    auto response = result.get();
    ASSERT_TRUE(response->success);
    EXPECT_NEAR(response->transform.transform.translation.z, 0.3, 1e-6);
    EXPECT_NEAR(response->transform.transform.rotation.w, 1.0, 1e-6);
}

TEST_F(TestSimpleTfKinematics, TestDynamicTransform) {
    // Allow time for several dynamic transforms to be published
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    auto client = node_->create_client<robot_interface::srv::GetTransform>("get_transform");
    
    auto request = std::make_shared<robot_interface::srv::GetTransform::Request>();
    request->frame_id = "odom";
    request->child_frame_id = "robot_base";
    
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    auto result = client->async_send_request(request);
    
    ASSERT_EQ(rclcpp::spin_until_future_complete(node_, result), 
              rclcpp::FutureReturnCode::SUCCESS);
              
    auto response = result.get();
    ASSERT_TRUE(response->success);
    // Check that x position is greater than initial position due to increments
    EXPECT_GT(response->transform.transform.translation.x, 0.0);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
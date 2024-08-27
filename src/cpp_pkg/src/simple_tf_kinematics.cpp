#include "simple_tf_kinematics.hpp"

using namespace std::chrono_literals;

SimpleTfKinematics::SimpleTfKinematics(const std::string &name) : Node(name), x_increment_(0.05), last_x_(0.0) {

    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    dynamic_tf_broadcaster_ = std::make_unique <tf2_ros::TransformBroadcaster>(*this);

    static_transform_stamped_.header.stamp = get_clock()->now();
    static_transform_stamped_.header.frame_id = "robot_base";
    static_transform_stamped_.child_frame_id = "robot_top";
    static_transform_stamped_.transform.translation.x = 0.0;
    static_transform_stamped_.transform.translation.y = 0.0;
    static_transform_stamped_.transform.translation.z = 0.3;
    static_transform_stamped_.transform.rotation.x = 0.0;
    static_transform_stamped_.transform.rotation.y = 0.0;
    static_transform_stamped_.transform.rotation.z = 0.0;
    static_transform_stamped_.transform.rotation.w = 1.0;

    static_tf_broadcaster_->sendTransform(static_transform_stamped_);

    RCLCPP_INFO_STREAM(get_logger(), "Publishing Static Transform between " << static_transform_stamped_.header.frame_id << "and" <<  static_transform_stamped_.child_frame_id);

    timer_ = create_wall_timer(0.1s, std::bind(&SimpleTfKinematics::timerCallback, this));

}

void SimpleTfKinematics::timerCallback()
{  
  dynamic_transform_stamped_.header.stamp = get_clock()->now();
  dynamic_transform_stamped_.header.frame_id = "odom";
  dynamic_transform_stamped_.child_frame_id = "robot_base";
  dynamic_transform_stamped_.transform.translation.x = last_x_ + x_increment_;
  dynamic_transform_stamped_.transform.translation.y = 0;
  dynamic_transform_stamped_.transform.translation.z = 0;
  // dynamic_transform_stamped_.transform.rotation.x = 0;
  // dynamic_transform_stamped_.transform.rotation.y = 0;
  // dynamic_transform_stamped_.transform.rotation.z = 0;
  // dynamic_transform_stamped_.transform.rotation.w = 1;

  dynamic_tf_broadcaster_->sendTransform(dynamic_transform_stamped_);
  last_x_ = dynamic_transform_stamped_.transform.translation.x;

}


int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleTfKinematics>("simple_tf_kinematics");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class TwistToTwistStampedNode : public rclcpp::Node
{
public:
  TwistToTwistStampedNode()
  : Node("twist_to_twiststamped_node")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&TwistToTwistStampedNode::topic_callback, this, std::placeholders::_1));
    
    publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/robot_controller/cmd_vel", 10);
  }

private:
  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    auto stamped_msg = geometry_msgs::msg::TwistStamped();
    stamped_msg.header.stamp = this->now();
    stamped_msg.twist = *msg;
    publisher_->publish(stamped_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistToTwistStampedNode>());
  rclcpp::shutdown();
  return 0;
}
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <chrono>
#include <string>

using namespace std::chrono_literals;

class LlamaController : public rclcpp::Node
{
public:
    LlamaController()
    : Node("llama_controller")
    {
        this->declare_parameter("prompt", "Move forward for 1 meter at a speed of 0.5 meters per second");
        prompt_ = this->get_parameter("prompt").as_string();

        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/robot_controller/cmd_vel", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&LlamaController::timer_callback, this));
        
        start_time_ = this->now();
        linear_speed_ = 0.5;  // 0.5 meters per second
        distance_ = 1.0;  // 1 meter
        duration_ = std::chrono::duration<double>(distance_ / linear_speed_);
        
        RCLCPP_INFO(this->get_logger(), "Llama Controller has been started");
        RCLCPP_INFO(this->get_logger(), "Prompt: %s", prompt_.c_str());
        RCLCPP_INFO(this->get_logger(), "Moving forward for %.2f meters at %.2f m/s", distance_, linear_speed_);
    }

private:
    void timer_callback()
    {
        auto current_time = this->now();
        auto elapsed_time = current_time - start_time_;
        
        if (elapsed_time.seconds() < duration_.count())
        {
            auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
            twist_msg->header.stamp = current_time;
            twist_msg->header.frame_id = "base_link";
            twist_msg->twist.linear.x = linear_speed_;
            twist_msg->twist.linear.y = 0.0;
            twist_msg->twist.linear.z = 0.0;
            twist_msg->twist.angular.x = 0.0;
            twist_msg->twist.angular.y = 0.0;
            twist_msg->twist.angular.z = 0.0;
            
            publisher_->publish(std::move(twist_msg));
            RCLCPP_INFO(this->get_logger(), "Published velocity: linear_x = %.2f m/s", linear_speed_);
        }
        else if (!movement_completed_)
        {
            // Stop the robot
            auto stop_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
            stop_msg->header.stamp = current_time;
            stop_msg->header.frame_id = "base_link";
            publisher_->publish(std::move(stop_msg));
            
            RCLCPP_INFO(this->get_logger(), "Movement completed. Robot stopped.");
            movement_completed_ = true;
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
    double linear_speed_;
    double distance_;
    std::chrono::duration<double> duration_;
    bool movement_completed_ = false;
    std::string prompt_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LlamaController>();
    
    // Print node info
    RCLCPP_INFO(node->get_logger(), "Node Info:");
    RCLCPP_INFO(node->get_logger(), "  Name: %s", node->get_name());
    RCLCPP_INFO(node->get_logger(), "  Namespace: %s", node->get_namespace());
    auto parameters = node->get_parameters(std::vector<std::string>{"prompt"});
    for (const auto& param : parameters) {
        RCLCPP_INFO(node->get_logger(), "  Parameter: %s = %s", param.get_name().c_str(), param.value_to_string().c_str());
    }
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
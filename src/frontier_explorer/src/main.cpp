#include "frontier_explorer/frontier_explorer.hpp"

/**
 * @brief Main entry point for the frontier explorer node
 * @param argc Number of command line arguments
 * @param argv Array of command line argument strings
 * @return Exit status code
 *
 * Initializes ROS2, creates a FrontierExplorer node, spins the node to process callbacks,
 * and performs cleanup on shutdown.
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrontierExplorer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
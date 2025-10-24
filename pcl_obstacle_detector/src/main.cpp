#include "pcl_obstacle_detector/pcl_obstacle_detector.hpp"

/// @brief Entry point for the PCL Obstacle Detector node
/// @param argc 
/// @param argv 
/// @return int Exit status
int main(int argc, char *argv[]) {
    // Main entry point
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCLNode>());
    rclcpp::shutdown();
    return 0;
}


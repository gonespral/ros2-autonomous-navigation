#include "pcl_obstacle_detector/pcl_obstacle_detector.hpp"

int main(int argc, char *argv[]) {
    // Main entry point for program. Initialize rclcpp and spin node.
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCLNode>());
    rclcpp::shutdown();
    return 0;
}


#include "control_barrel_world/control_barrel_world.hpp"

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CBWNode>());
    rclcpp::shutdown();
    return 0;
}



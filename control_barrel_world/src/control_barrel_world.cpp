#include "control_barrel_world/control_barrel_world.hpp"
#include <string>

class CBWNode: public rclcpp::Node {
    public:
        CBWNode(): Node("control_barrel_world_node")
        {

            // Get node args and set defaults
            this->declare_parameter("pedestrians_topic", "/pedestrians");  // vision_msgs/msg/Detection2DArray
            this->declare_parameter("detections_topic", "/detections");  // vision_msgs/msg/Detection3DArray
            this->declare_parameter("publish_interval_ms", 100);
            this->declare_parameter("queue_size", 1);

            // Read parameter values into class members
            pedestrians_topic = this->get_parameter("pedestrians_topic").as_string();
            detections_topic = this->get_parameter("detections_topic").as_string();
            publish_interval_ms = this->get_parameter("publish_interval_ms").as_int();
            queue_size = this->get_parameter("queue_size").as_int();

            // Log info
            RCLCPP_INFO(this->get_logger(), "Starting node '%s'...", this->get_name());

            // Create subscriber and publisher
            subscription_pedestrians = 
            this->create_subscription<vision_msgs::msg::Detection2DArray>(
                pedestrians_topic, 
                queue_size,
                std::bind(&CBWNode::callback_pededstrians, this, std::placeholders::_1) // bind is needed to create callable object, since member func has implied "this" as first arg
            );

            subscription_detections = 
            this->create_subscription<vision_msgs::msg::Detection3DArray>(
                detections_topic,
                queue_size,
                std::bind(&CBWNode::callback_detections, this, std::placeholders::_1) 
            );

            // Log info
            RCLCPP_INFO(
                this->get_logger(),
                "Started node '%s'\nPedestrians topic: '%s'\nDetections topic: '%s'\nPublish interval: %d ms\nQueue size: %d",
                this->get_name(),
                pedestrians_topic.c_str(),
                detections_topic.c_str(),
                publish_interval_ms,
                queue_size
            );

        };

    private:
        // Declare node configuration parameters - parameters stored as class members
        std::string pedestrians_topic;
        std::string detections_topic;
        int publish_interval_ms;
        int queue_size;

        // Declare subscriber and publisher
        rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr subscription_pedestrians;
        rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr subscription_detections;

        // Define callback function
        int callback_pededstrians(const vision_msgs::msg::Detection2DArray msg) {
            RCLCPP_INFO(this->get_logger(), "PED - Received %zu pedestrian detections", msg.detections.size());
            return 0;
        }

        int callback_detections(const vision_msgs::msg::Detection3DArray msg) {
            RCLCPP_INFO(this->get_logger(), "DET - Received %zu 3D detections", msg.detections.size());
            return 0;
        }
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CBWNode>());
    rclcpp::shutdown();
    return 0;
}



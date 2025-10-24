#ifndef CONTROL_BARREL_WORLD
#define CONTROL_BARREL_WORLD

// From standard library
#include <string>
#include <chrono>
#include <memory>

// From ROS
#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// From TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class CBWNode : public rclcpp::Node
{
    public:
        CBWNode();

    private:

        // Declare subscriber and publisher
        rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr subscription_pedestrians;
        rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr subscription_detections;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_command;

        // Declare timer
        rclcpp::TimerBase::SharedPtr timer;

        // Declare TF buffer and listener
        std::shared_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;

        // Declare callback functions
        int pedestriansCallback(const vision_msgs::msg::Detection2DArray::UniquePtr msg);
        int detectionsCallback(const vision_msgs::msg::Detection3DArray::UniquePtr msg);
        void timerCallback();

        // Declare node configuration parameters - parameters stored as class members
        std::string pedestrians_topic;
        std::string detections_topic;
        std::string command_topic;
        int publish_interval_ms;
        int queue_size;
        std::vector<vision_msgs::msg::Detection2D> latest_pedestrians;
        std::vector<vision_msgs::msg::Detection3D> latest_detections;
        double detection_radius;
        double pedestrian_area_thresh;
        double linear_speed;
        double angular_speed;


};

#endif
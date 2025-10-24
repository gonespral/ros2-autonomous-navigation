#include "control_barrel_world/control_barrel_world.hpp"

// Declare node constructor function, inherited class from rclcpp::Node
CBWNode::CBWNode(): Node("control_barrel_world_node")
{
    // Initialize TF buffer and listener
    tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Get node args and set defaults
    this->declare_parameter("pedestrians_topic", "/pedestrians");  // vision_msgs/msg/Detection2DArray
    this->declare_parameter("detections_topic", "/detections");  // vision_msgs/msg/Detection3DArray
    this->declare_parameter("command_topic", "/mirte/cmd_vel"); // geometry_msgs/msg/Twist
    this->declare_parameter("publish_interval_ms", 1);
    this->declare_parameter("queue_size", 1);
    this->declare_parameter("detection_radius", 0.7);
    this->declare_parameter("pedestrian_area_thresh", 2500.0);
    this->declare_parameter("linear_speed", 0.17);
    this->declare_parameter("angular_speed", 0.6);

    // Read parameter values into class members
    pedestrians_topic = this->get_parameter("pedestrians_topic").as_string();
    detections_topic = this->get_parameter("detections_topic").as_string();
    command_topic = this->get_parameter("command_topic").as_string();
    publish_interval_ms = this->get_parameter("publish_interval_ms").as_int();
    queue_size = this->get_parameter("queue_size").as_int();
    detection_radius = this->get_parameter("detection_radius").as_double();
    pedestrian_area_thresh = this->get_parameter("pedestrian_area_thresh").as_double();
    linear_speed = this->get_parameter("linear_speed").as_double();
    angular_speed = this->get_parameter("angular_speed").as_double();

    // Log info
    RCLCPP_INFO(this->get_logger(), "Starting node '%s'...", this->get_name());

    // Create subscriber and publisher
    subscription_pedestrians = 
    this->create_subscription<vision_msgs::msg::Detection2DArray>(
        pedestrians_topic, 
        queue_size,
        std::bind(&CBWNode::pedestriansCallback, this, std::placeholders::_1) // bind is needed to create callable object, since member func has implied "this" as first arg
    );

    subscription_detections = 
    this->create_subscription<vision_msgs::msg::Detection3DArray>(
        detections_topic,
        queue_size,
        std::bind(&CBWNode::detectionsCallback, this, std::placeholders::_1) 
    );

    publisher_command = 
    this->create_publisher<geometry_msgs::msg::Twist>(
        command_topic,
        queue_size
    );

    // Create timer for publishing control commands
    timer =
    this->create_wall_timer(
        std::chrono::milliseconds(publish_interval_ms),
        std::bind(&CBWNode::timerCallback, this)
    );

    // Log info
    RCLCPP_INFO(
        this->get_logger(),
        "Started node '%s'\nPedestrians topic: '%s'\nDetections topic: '%s'\nCommand topic: '%s'\nPublish interval: %d ms\nQueue size: %d",
        this->get_name(),
        pedestrians_topic.c_str(),
        detections_topic.c_str(),
        command_topic.c_str(),
        publish_interval_ms,
        queue_size
    );

};



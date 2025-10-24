#include "pcl_obstacle_detector/pcl_obstacle_detector.hpp"


// Declare node constructor function, inherited class from rclcpp::Node
PCLNode::PCLNode(): Node("pcl_obstacle_detector_node")
{
    // Get node args and set defaults
    this->declare_parameter("point_cloud_topic", "/mirte/camera_depth/points");
    this->declare_parameter("detections_topic", "/detections");
    this->declare_parameter("publish_interval_ms", 100);
    this->declare_parameter("queue_size", 1);
    this->declare_parameter("filter_limit_l", 0.1);
    this->declare_parameter("filter_limit_u", 3.0);
    this->declare_parameter("segmentation_thresh", 0.01);
    this->declare_parameter("downsample_tolerance", 0.01);
    this->declare_parameter("cluster_tolerance", 0.01);
    this->declare_parameter("cluster_size_min", 5);
    this->declare_parameter("cluster_size_max", 25000);

    // Read parameter values into class members
    point_cloud_topic = this->get_parameter("point_cloud_topic").as_string();
    detections_topic = this->get_parameter("detections_topic").as_string();
    publish_interval_ms = this->get_parameter("publish_interval_ms").as_int();
    queue_size = this->get_parameter("queue_size").as_int();
    filter_limit_l = this->get_parameter("filter_limit_l").as_double();
    filter_limit_u = this->get_parameter("filter_limit_u").as_double();
    segmentation_thresh = this->get_parameter("segmentation_thresh").as_double();
    segmentation_thresh = this->get_parameter("downsample_tolerance").as_double();
    cluster_tolerance = this->get_parameter("cluster_tolerance").as_double();
    cluster_size_min = this->get_parameter("cluster_size_min").as_int();
    cluster_size_max = this->get_parameter("cluster_size_max").as_int();

    // Log info
    RCLCPP_INFO(this->get_logger(), "Starting node '%s'...", this->get_name());

    // Create subscriber and publisher
    subscription_point_cloud = 
    this->create_subscription<sensor_msgs::msg::PointCloud2>(
        point_cloud_topic, 
        queue_size,
        std::bind(&PCLNode::pointCloudCallback, this, std::placeholders::_1)
        // Passes msg to pointCloudCallback as shared pointer stored in heap
    );

    publisher_detections = 
    this->create_publisher<vision_msgs::msg::Detection3DArray>(
        detections_topic,
        queue_size
    );

    // Log node started
    RCLCPP_INFO(
        this->get_logger(),
        "Started node '%s'\nPoint cloud topic: '%s'\nDetections topic: '%s'\nPublish interval: %d ms\nQueue size: %d",
        this->get_name(),
        point_cloud_topic.c_str(),
        detections_topic.c_str(),
        publish_interval_ms,
        queue_size
    );
}

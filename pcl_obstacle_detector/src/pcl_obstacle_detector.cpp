#include "pcl_obstacle_detector/pcl_obstacle_detector.hpp"
#include <string>

class PCLNode: public rclcpp::Node {
    public:
        PCLNode(): Node("pcl_obstacle_detector_node")
        {

            // Get node args and set defaults
            this->declare_parameter("point_cloud_topic", "/mirte/camera_depth/points");
            this->declare_parameter("detections_topic", "/detections");
            this->declare_parameter("publish_interval_ms", 100);
            this->declare_parameter("queue_size", 1);

            // Declare node configuration parameters
            std::string point_cloud_topic = this->get_parameter("point_cloud_topic").as_string();  // sensor_msgs/msg/PointCloud2
            std::string detections_topic = this->get_parameter("detections_topic").as_string();  // vision_msgs/msg/Detection3DArray
            int publish_interval_ms = this->get_parameter("publish_interval_ms").as_int();
            int queue_size = this->get_parameter("queue_size").as_int();

            // Log info
            RCLCPP_INFO(this->get_logger(), "Starting node '%s'...", this->get_name());

            // Create subscriber and publisher
            subscription_point_cloud = 
            this->create_subscription<sensor_msgs::msg::PointCloud2>(
                point_cloud_topic, 
                queue_size,
                std::bind(&PCLNode::callback_point_cloud, this, std::placeholders::_1) // bind is needed to create callable object, since member func has implied "this" as first arg
            );

            publisher_detections = 
            this->create_publisher<vision_msgs::msg::Detection3DArray>(
                detections_topic,
                queue_size
            );

            // Log info
            RCLCPP_INFO(
                this->get_logger(),
                "Started node '%s'\nPoint cloud topic: '%s'\nDetections topic: '%s'\nPublish interval: %d ms\nQueue size: %d",
                this->get_name(),
                point_cloud_topic.c_str(),
                detections_topic.c_str(),
                publish_interval_ms,
                queue_size
            );

        };

    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_point_cloud;
        rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr publisher_detections;

        // Define callback functions
        void callback_point_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
            // RCLCPP_INFO(
            //     this->get_logger(),
            //     "Received PointCloud2: frame_id='%s', width=%u, height=%u, fields=%zu",
            //     msg->header.frame_id.c_str(),
            //     msg->width,
            //     msg->height,
            //     msg->fields.size()
            // );

            // Declare cloud pariables
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

            // Convert from sensor_msgs::msg::PointCloud2 to pcl::PointCloud
            pcl::fromROSMsg(*msg, *cloud);  // msg -> cloud

            RCLCPP_INFO(this->get_logger(), "Raw point cloud size: %zu", cloud->size());

            // Remove NaN points
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*cloud, *cloud, indices); // cloud -> cloud

            // Filter points too high
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(cloud);
            pass.setFilterFieldName("z");  // filter axis
            pass.setFilterLimits(0.1, 3.0); // keep points between 0.1m and 0.3m
            pass.filter(*cloud_filtered);  // store in cloud_filtered

            RCLCPP_INFO(this->get_logger(), "Filtered point cloud size: %zu", cloud->size());

        // TODO: Convert to detections later
        // vision_msgs::msg::Detection3DArray detections;
        // publisher_detections_->publish(detections);
    }
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCLNode>());
    rclcpp::shutdown();
    return 0;
}


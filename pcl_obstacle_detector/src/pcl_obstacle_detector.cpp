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
            this->declare_parameter("filter_limit_l", 0.1);
            this->declare_parameter("filter_limit_u", 3.0);
            this->declare_parameter("segmentation_thres", 0.01);

            // Read parameter values into class members
            point_cloud_topic = this->get_parameter("point_cloud_topic").as_string();
            detections_topic = this->get_parameter("detections_topic").as_string();
            publish_interval_ms = this->get_parameter("publish_interval_ms").as_int();
            queue_size = this->get_parameter("queue_size").as_int();
            filter_limit_l = this->get_parameter("filter_limit_l").as_double();
            filter_limit_u = this->get_parameter("filter_limit_u").as_double();
            segmentation_thresh = this->get_parameter("segmentation_thresh").as_double();

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
        // Declare subscriber and publisher
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_point_cloud;
        rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr publisher_detections;

        // Declare node configuration parameters - parameters stored as class members
        std::string point_cloud_topic;
        std::string detections_topic;
        int publish_interval_ms;
        int queue_size;
        double filter_limit_l;
        double filter_limit_u;
        double segmentation_thresh;

        // Define callback functions
        int callback_point_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {

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
            pass.setFilterLimits(this->filter_limit_l, this->filter_limit_u); // keep points between 0.1m and 0.3m
            pass.filter(*cloud_filtered);  // store in cloud_filtered

            RCLCPP_INFO(this->get_logger(), "Filtered point cloud size: %zu", cloud->size());

            // Ground plane removal
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

            // Create the segmentation object
            pcl::SACSegmentation<pcl::PointXYZ> seg;

            // Optional
            seg.setOptimizeCoefficients(true);

            // Mandatory
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.01);

            seg.setInputCloud(cloud_filtered);
            seg.segment(*inliers, *coefficients);

            if (inliers->indices.size() == 0) {
                RCLCPP_INFO(this->get_logger(), "Could not estimate a planar model for the given dataset.\n");
                return -1;
            }

            return 0;


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


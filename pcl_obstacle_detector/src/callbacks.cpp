#include "pcl_obstacle_detector/pcl_obstacle_detector.hpp"

int PCLNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {

    // Assign memory to cloud and cloud_filtered
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    // Pre-process point cloud
    this->preprocessPointCloud(msg, cloud, cloud_filtered);

    // Remove ground plane
    this->removeGroundPlane(cloud_filtered);

    // Cluster detections and generate messages
    auto detections = this->euclideanClusterExtraction(cloud_filtered, msg);

    // Publish detections
    publisher_detections->publish(detections);

    return 0;
}
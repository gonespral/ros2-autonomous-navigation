#include "pcl_obstacle_detector/pcl_obstacle_detector.hpp"

// Define callback for PCLNode. Called for every message in point cloud topic, to process it.
// Publishes message on detections, with processed data.
int PCLNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {

    // Assign memory to cloud and cloud_filtered.
    // Instead of using return values for some functions, perform modifications on pointed-to data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    // Pre-process point cloud
    this->preprocessPointCloud(msg, cloud, cloud_filtered);

    // Remove ground plane
    this->removeGroundPlane(cloud_filtered);

    // Cluster detections and output detections message
    // Need to pass msg pointer so that it can extract the header
    auto detections = this->euclideanClusterExtraction(cloud_filtered, msg);

    // Publish detections
    publisher_detections->publish(detections);

    return 0;
}
#include "pcl_obstacle_detector/pcl_obstacle_detector.hpp"

void PCLNode::preprocessPointCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered
) {
    
    // Convert from sensor_msgs::msg::PointCloud2 to pcl::PointCloud
    pcl::fromROSMsg(*msg, *cloud);  // dereference to access data at msg and cloud

    RCLCPP_INFO(this->get_logger(), "RAW CLOUD - Raw point cloud size: %zu", cloud->size());

    // Remove NaN points
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    // PassThrough filter on z
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(this->filter_limit_l, this->filter_limit_u);
    pass.filter(*cloud_filtered); // store in cloud filtered

    RCLCPP_INFO(this->get_logger(), "FILTERING - Filtered point cloud size: %zu", cloud_filtered->size());
}

void PCLNode::removeGroundPlane(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered
) {

    // Allocate memory for coefficients and inliers
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // Create segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(this->segmentation_thresh);

    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);

    RCLCPP_INFO(this->get_logger(), "GROUND SEGMENTATION - Plane inliers size: %zu", inliers->indices.size());

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(true);  // keep non-ground
    extract.filter(*cloud_filtered);

    RCLCPP_INFO(this->get_logger(), "GROUND SEGMENTATION - Filtered point cloud size: %zu", cloud_filtered->size());
}

vision_msgs::msg::Detection3DArray PCLNode::euclideanClusterExtraction(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, 
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg
) {

    // Allocate memory to KdTree object
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>); 
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

    ec.setClusterTolerance(this->cluster_tolerance);
    ec.setMinClusterSize(this->cluster_size_min);
    ec.setMaxClusterSize(this->cluster_size_max);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    vision_msgs::msg::Detection3DArray detections;
    detections.header = msg->header;

    for (const auto& cluster : cluster_indices) {
        // Allocate memory to new cloud cluster
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

        // Iterate over cluster indices and append index to cloud_cluster
        for (const auto& idx : cluster.indices) {
            cloud_cluster->push_back((*cloud_filtered)[idx]);
        }

        // We now have cloud cluster with points corresponding to that cluster
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // Compute centroid
        Eigen::Matrix<double, 4, 1> centroid;
        pcl::compute3DCentroid(*cloud_cluster, centroid);

        // Compute maximum and minimum 3D points
        pcl::PointXYZ min_pt;
        pcl::PointXYZ max_pt;
        pcl::getMinMax3D(*cloud_cluster, min_pt, max_pt);

        // Create detection message
        vision_msgs::msg::Detection3D detection;
        detection.header = msg->header;

        // Add position of bounding box
        detection.bbox.center.position.x = centroid[0];
        detection.bbox.center.position.y = centroid[1];
        detection.bbox.center.position.z = centroid[2];
        detection.bbox.center.orientation.w = 1.0;

        // Add size of bounding box
        detection.bbox.size.x = max_pt.x - min_pt.x;
        detection.bbox.size.y = max_pt.y - min_pt.y;
        detection.bbox.size.z = max_pt.z - min_pt.z;

        // Add detection to array
        detections.detections.push_back(detection);
    }

    return detections;
}
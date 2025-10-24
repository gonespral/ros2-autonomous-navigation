#ifndef PCL_OBSTACLE_DETECTOR
#define PCL_OBSTACLE_DETECTOR

// From standard library
#include <string>

// From mirte package
#include "sensor_msgs/msg/point_cloud2.hpp"

// From ROS
#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>

// From PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

// PCL ROS Package includes
#include <pcl_conversions/pcl_conversions.h>

class PCLNode : public rclcpp::Node
{
public:
    PCLNode();

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_point_cloud;
    
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr publisher_detections;
    
    int pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

    void preprocessPointCloud(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg,
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered
    );

    void removeGroundPlane(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered
    );

    vision_msgs::msg::Detection3DArray euclideanClusterExtraction(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, 
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg
    ); 

    // Declare node configuration parameters - parameters stored as class members
    std::string point_cloud_topic;
    std::string detections_topic;
    int publish_interval_ms;
    int queue_size;
    double filter_limit_l;
    double filter_limit_u;
    double segmentation_thresh;
    double downsample_tolerance;
    double cluster_tolerance;
    int cluster_size_min;
    int cluster_size_max;
};

#endif

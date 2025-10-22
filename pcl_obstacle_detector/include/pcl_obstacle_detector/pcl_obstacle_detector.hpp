#ifndef PCL_OBSTACLE_DETECTOR
#define PCL_OBSTACLE_DETECTOR

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <vision_msgs/msg/detection3_d_array.hpp>

// PCL includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// PCL ROS Package includes
#include <pcl_conversions/pcl_conversions.h>


#endif

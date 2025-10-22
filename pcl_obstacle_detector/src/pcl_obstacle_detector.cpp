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
            this->declare_parameter("segmentation_thresh", 0.01);
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
        double cluster_tolerance;
        int cluster_size_min;
        int cluster_size_max;

        // Define callback functions
        int callback_point_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {

            // Declare cloud pariables
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

            // Convert from sensor_msgs::msg::PointCloud2 to pcl::PointCloud
            pcl::fromROSMsg(*msg, *cloud);  // msg -> cloud

            RCLCPP_INFO(this->get_logger(), "RAW CLOUD - Raw point cloud size: %zu", cloud->size());

            // Remove NaN points
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*cloud, *cloud, indices); // cloud -> cloud

            // Filter points too high
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(cloud);
            pass.setFilterFieldName("z");  // filter axis
            pass.setFilterLimits(this->filter_limit_l, this->filter_limit_u); // keep points between 0.1m and 0.3m
            pass.filter(*cloud_filtered);  // store in cloud_filtered

            RCLCPP_INFO(this->get_logger(), "FILTERING - Filtered point cloud size: %zu", cloud->size());

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

            RCLCPP_INFO(this->get_logger(), "GROUND SEGMENTATION - Plane inliers size: %zu", inliers->indices.size());
            
        
            // // Create the filtering object: downsample the dataset using a leaf size of 1cm
            // pcl::VoxelGrid<pcl::PointXYZ> vg;
            // vg.setInputCloud (cloud);
            // vg.setLeafSize (0.01f, 0.01f, 0.01f);
            // vg.filter (*cloud_filtered);
            // std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //*

            // // Create the segmentation object for the planar model and set all the parameters
            // pcl::SACSegmentation<pcl::PointXYZ> seg;
            // pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
            // pcl::PCDWriter writer;
            // seg.setOptimizeCoefficients (true);
            // seg.setModelType (pcl::SACMODEL_PLANE);
            // seg.setMethodType (pcl::SAC_RANSAC);
            // seg.setMaxIterations (100);
            // seg.setDistanceThreshold (0.02);

            // int nr_points = (int) cloud_filtered->size ();
            // while (cloud_filtered->size () > 0.3 * nr_points)
            // {
            //     // Segment the largest planar component from the remaining cloud
            //     seg.setInputCloud (cloud_filtered);
            //     seg.segment (*inliers, *coefficients);
            //     if (inliers->indices.size () == 0)
            //     {
            //     std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            //     break;
            //     }

            //     // Extract the planar inliers from the input cloud
            //     pcl::ExtractIndices<pcl::PointXYZ> extract;
            //     extract.setInputCloud (cloud_filtered);
            //     extract.setIndices (inliers);
            //     extract.setNegative (false);

            //     // Get the points associated with the planar surface
            //     extract.filter (*cloud_plane);
            //     std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;

            //     // Remove the planar inliers, extract the rest
            //     extract.setNegative (true);
            //     extract.filter (*cloud_f);
            //     *cloud_filtered = *cloud_f;
            // }

            // // Creating the KdTree object for the search method of the extraction
            // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
            // tree->setInputCloud (cloud_filtered);

            // std::vector<pcl::PointIndices> cluster_indices;
            // pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            // ec.setClusterTolerance (0.02); // 2cm
            // ec.setMinClusterSize (100);
            // ec.setMaxClusterSize (25000);
            // ec.setSearchMethod (tree);
            // ec.setInputCloud (cloud_filtered);
            // ec.extract (cluster_indices);

            // int j = 0;
            // for (const auto& cluster : cluster_indices)
            // {
            //     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
            //     for (const auto& idx : cluster.indices) {
            //     cloud_cluster->push_back((*cloud_filtered)[idx]);
            //     } //*
            //     cloud_cluster->width = cloud_cluster->size ();
            //     cloud_cluster->height = 1;
            //     cloud_cluster->is_dense = true;

            //     std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
            //     std::stringstream ss;
            //     ss << std::setw(4) << std::setfill('0') << j;
            //     writer.write<pcl::PointXYZ> ("cloud_cluster_" + ss.str () + ".pcd", *cloud_cluster, false); //*
            //     j++;
            // }





        // TODO: Convert to detections later
        // vision_msgs::msg::Detection3DArray detections;
        // publisher_detections_->publish(detections);

        // TODO: Refactor postproc into separate file
            return 0;
    }
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCLNode>());
    rclcpp::shutdown();
    return 0;
}


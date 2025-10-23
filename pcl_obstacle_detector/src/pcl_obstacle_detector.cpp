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

        // Declare subscriber and publisher
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_point_cloud;
        rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr publisher_detections;

        // Define callback function - process point cloud
        int callback_point_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {

            // Declare cloud variables
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

            // Convert from sensor_msgs::msg::PointCloud2 to pcl::PointCloud
            pcl::fromROSMsg(*msg, *cloud);  // msg -> cloud

            RCLCPP_INFO(this->get_logger(), "RAW CLOUD - Raw point cloud size: %zu", cloud->size());

            // -- 1. Pre-Process (Filter) Point Cloud --
            // https://pcl.readthedocs.io/projects/tutorials/en/master/passthrough.html
            
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

            // -- 2. Ground Plane Removal --
            // https://pcl.readthedocs.io/projects/tutorials/en/master/planar_segmentation.html
            // https://pcl.readthedocs.io/projects/tutorials/en/master/extract_indices.html#extract-indices

            // Ground plane removal
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

            // Create the segmentation object
            pcl::SACSegmentation<pcl::PointXYZ> seg;

            // Optional
            seg.setOptimizeCoefficients(true);

            // Mandatory
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(this->segmentation_thresh);

            seg.setInputCloud(cloud_filtered);
            seg.segment(*inliers, *coefficients);

            if (inliers->indices.size() == 0) {
                RCLCPP_INFO(this->get_logger(), "Could not estimate a planar model for the given dataset.\n");
                return -1;
            }

            RCLCPP_INFO(this->get_logger(), "GROUND SEGMENTATION - Plane inliers size: %zu", inliers->indices.size());

            // Before filtering, extract the ground plane
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud_filtered);  // give it value, not reference
            extract.setIndices(inliers);
            extract.setNegative(true);  // true to keep points that are not part of ground plane
            extract.filter(*cloud_filtered);  // now we have an updated cloud_filtered without ground plane

            RCLCPP_INFO(this->get_logger(), "GROUND SEGMENTATION - Filtered point cloud size: %zu", cloud_filtered->size());

            // -- 3. Euclidian Cluster Extraction --
            // https://pcl.readthedocs.io/projects/tutorials/en/master/cluster_extraction.html
            // https://pcl.readthedocs.io/projects/tutorials/en/master/voxel_grid.html#downsampling-a-pointcloud-using-a-voxelgrid-filter
        
            // Downsample using voxel grid
            // pcl::VoxelGrid<pcl::PointXYZ> vg;
            // vg.setInputCloud(cloud_filtered);  // take value of cloud_filtered
            // vg.setLeafSize(0.01f, 0.01f, 0.01f);
            // vg.filter(*cloud_filtered);

            // RCLCPP_INFO(this->get_logger(), "VOXEL GRID DOWNSAMPLE - Filtered point cloud size: %zu", cloud_filtered->size());
        
            // Create the segmentation object for the planar model and set all the parameters

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ> ());

            // Creating the KdTree object for the search method of the extraction
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);  // create new tree
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
                // Create new cloud_cluster
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

                // Iterate over cluster indices and append index to cloud_cluster
                for (const auto& idx : cluster.indices) {
                    cloud_cluster->push_back((*cloud_filtered)[idx]);
                } 

                // We now have cloud cluster with points corresponding to that cluster
                cloud_cluster->width = cloud_cluster->size();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;

                // We now have clusters in cluster_indices associated with cloud_filtered!
                // RCLCPP_INFO(this->get_logger(), "CLUSTERING - Cluster point cloud size: %zu", cloud_cluster->size());

                // Compute centroid
                // compute3DCentroid takes Eigen::Matrix<Scalar, 4, 1> &centroid
                Eigen::Matrix<double, 4, 1> centroid;  // 4x4 matrix to hold centroid
                pcl::compute3DCentroid(*cloud_cluster, centroid);

                // Compute maximum and minimum 3D points
                pcl::PointXYZ min_pt;
                pcl::PointXYZ max_pt;
                pcl::getMinMax3D(*cloud_cluster, min_pt, max_pt);

                // Create detection message
                vision_msgs::msg::Detection3D detection;
                detection.header = msg->header;

                // Position (center of bbox)
                detection.bbox.center.position.x = centroid[0];
                detection.bbox.center.position.y = centroid[1];
                detection.bbox.center.position.z = centroid[2];
                detection.bbox.center.orientation.w = 1.0;

                // Size of bounding box (width, height, depth)
                detection.bbox.size.x = max_pt.x - min_pt.x;
                detection.bbox.size.y = max_pt.y - min_pt.y;
                detection.bbox.size.z = max_pt.z - min_pt.z;

                // Add detection to array
                detections.detections.push_back(detection);
            }
    
            // Publish detections
            publisher_detections->publish(detections);

            return 0;
    }
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCLNode>());
    rclcpp::shutdown();
    return 0;
}


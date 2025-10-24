#include "control_barrel_world/control_barrel_world.hpp"
#include <string>
#include <chrono>

class CBWNode: public rclcpp::Node {
    public:
        CBWNode(): Node("control_barrel_world_node")
        {

            // Initialize TF buffer and listener
            tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

            // Get node args and set defaults
            this->declare_parameter("pedestrians_topic", "/pedestrians");  // vision_msgs/msg/Detection2DArray
            this->declare_parameter("detections_topic", "/detections");  // vision_msgs/msg/Detection3DArray
            this->declare_parameter("command_topic", "/mirte/cmd_vel"); // geometry_msgs/msg/Twist
            this->declare_parameter("publish_interval_ms", 1);
            this->declare_parameter("queue_size", 1);
            this->declare_parameter("detection_radius", 0.7);
            this->declare_parameter("pedestrian_area_thresh", 2500.0);
            this->declare_parameter("linear_speed", 0.17);
            this->declare_parameter("angular_speed", 0.6);

            // Read parameter values into class members
            pedestrians_topic = this->get_parameter("pedestrians_topic").as_string();
            detections_topic = this->get_parameter("detections_topic").as_string();
            command_topic = this->get_parameter("command_topic").as_string();
            publish_interval_ms = this->get_parameter("publish_interval_ms").as_int();
            queue_size = this->get_parameter("queue_size").as_int();
            detection_radius = this->get_parameter("detection_radius").as_double();
            pedestrian_area_thresh = this->get_parameter("pedestrian_area_thresh").as_double();
            linear_speed = this->get_parameter("linear_speed").as_double();
            angular_speed = this->get_parameter("angular_speed").as_double();

            // Log info
            RCLCPP_INFO(this->get_logger(), "Starting node '%s'...", this->get_name());

            // Create subscriber and publisher
            subscription_pedestrians = 
            this->create_subscription<vision_msgs::msg::Detection2DArray>(
                pedestrians_topic, 
                queue_size,
                std::bind(&CBWNode::callback_pededstrians, this, std::placeholders::_1) // bind is needed to create callable object, since member func has implied "this" as first arg
            );

            subscription_detections = 
            this->create_subscription<vision_msgs::msg::Detection3DArray>(
                detections_topic,
                queue_size,
                std::bind(&CBWNode::callback_detections, this, std::placeholders::_1) 
            );

            publisher_command = 
            this->create_publisher<geometry_msgs::msg::Twist>(
                command_topic,
                queue_size
            );

            // Create timer for publishing control commands
            timer =
            this->create_wall_timer(
                std::chrono::milliseconds(publish_interval_ms),
                std::bind(&CBWNode::callback_timer, this)
            );

            // Log info
            RCLCPP_INFO(
                this->get_logger(),
                "Started node '%s'\nPedestrians topic: '%s'\nDetections topic: '%s'\nCommand topic: '%s'\nPublish interval: %d ms\nQueue size: %d",
                this->get_name(),
                pedestrians_topic.c_str(),
                detections_topic.c_str(),
                command_topic.c_str(),
                publish_interval_ms,
                queue_size
            );

        };

    private:
        // Declare node configuration parameters - parameters stored as class members
        std::string pedestrians_topic;
        std::string detections_topic;
        std::string command_topic;
        int publish_interval_ms;
        int queue_size;
        std::vector<vision_msgs::msg::Detection2D> latest_pedestrians;
        std::vector<vision_msgs::msg::Detection3D> latest_detections;
        double detection_radius;
        double pedestrian_area_thresh;
        double linear_speed;
        double angular_speed;

        // Declare subscriber and publisher
        rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr subscription_pedestrians;
        rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr subscription_detections;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_command;

        // Declare timer
        rclcpp::TimerBase::SharedPtr timer;

        // TF buffer and listener
        std::shared_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;

        // Define callback function for logging
        int callback_pededstrians(const vision_msgs::msg::Detection2DArray::UniquePtr msg) {
            // RCLCPP_INFO(this->get_logger(), "PED - Received %zu pedestrian detections", msg->detections.size());
            latest_pedestrians = msg->detections;
            return 0;
        }

        int callback_detections(const vision_msgs::msg::Detection3DArray::UniquePtr msg) {
            // Store latest detections
            latest_detections = msg->detections;

            // Lookup transform from detections frame to base_link first
            geometry_msgs::msg::TransformStamped transform_stamped;
            try {
                transform_stamped = tf_buffer->lookupTransform(
                    "base_link",
                    msg->header.frame_id,
                    tf2::TimePointZero);
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
                return 0;
            }

            // Transform each detection position into base_link and store back into latest_detections
            for (auto &det : latest_detections) {
                geometry_msgs::msg::PointStamped pos_in;
                geometry_msgs::msg::PointStamped pos_out;
                pos_in.header.frame_id = msg->header.frame_id;
                pos_in.header.stamp = msg->header.stamp;
                pos_in.point = det.bbox.center.position;

                tf2::doTransform(pos_in, pos_out, transform_stamped);

                det.bbox.center.position = pos_out.point;
            }

            return 0;
        }

        void callback_timer() {
            // Take most recent detections, process and publish to command topic
            auto twist_msg = geometry_msgs::msg::Twist();

            // Action variable 0 = left, 1 = forward, 2 = right, 3 = stop
            int action = 1;
            
            // Filter detections
            std::vector<vision_msgs::msg::Detection2D> latest_pedestrians_proc;
            std::vector<vision_msgs::msg::Detection3D> latest_detections_proc;
            std::vector<double> latest_detections_distances;

            for (const auto& detection : latest_detections) {
                double pos_x = detection.bbox.center.position.x;
                double pos_y = detection.bbox.center.position.y;
                double pos_z = detection.bbox.center.position.z;
                double distance = std::sqrt(pos_x * pos_x + pos_y * pos_y + pos_z * pos_z);

                // If detection in front of robot
                if (pos_x > 0) {
                    // If detection within radius
                    if (distance < detection_radius) {
                        latest_detections_proc.push_back(detection);
                        latest_detections_distances.push_back(distance);
                    }
                }
            }

            // RCLCPP_INFO(this->get_logger(), "Filtered detections array size %zu", latest_detections_proc.size());

            // Decide action
            if (latest_detections_proc.size() == 0) {
                action = 1;
                RCLCPP_INFO(this->get_logger(), "No obstacles detected, driving forward");
            } else {
                // Get closest object 
                int closest_index = std::min_element(latest_detections_distances.begin(), latest_detections_distances.end()) - latest_detections_distances.begin();

                // If closest object is to the left, drive right. Else left.
                if (latest_detections_proc[closest_index].bbox.center.position.y > 0) {
                    action = 2;
                    RCLCPP_INFO(this->get_logger(), "Obstacle detected on left, driving right");
                } else {
                    action = 0;
                    RCLCPP_INFO(this->get_logger(), "Obstacle detected on right, driving left");
                }
            }

            // Check if person detected with area > 20000 pixels2
            for (const auto& pedestrian : latest_pedestrians) {
                double size_x = pedestrian.bbox.size_x;
                double size_y = pedestrian.bbox.size_y;
                double area = size_x * size_y;

                if (area > pedestrian_area_thresh) {
                    action = 3;
                    break;
                }
            }

            switch(action) {
                case 0:
                    // left
                    twist_msg.linear.x = linear_speed;
                    twist_msg.angular.z = angular_speed;
                    RCLCPP_INFO(this->get_logger(), "Driving left");
                    break;
                case 1:
                    // forward
                    twist_msg.linear.x = linear_speed;
                    twist_msg.angular.z = 0;
                    RCLCPP_INFO(this->get_logger(), "Driving forward");
                    break;
                case 2: 
                    // right
                    twist_msg.linear.x = linear_speed;
                    twist_msg.angular.z = -angular_speed;
                    RCLCPP_INFO(this->get_logger(), "Driving right");
                    break;
                case 3: 
                    // stop
                    twist_msg.linear.x = 0;
                    twist_msg.angular.z = 0;
                    RCLCPP_INFO(this->get_logger(), "Stopping");
                    break;
                default:
                    //stop
                    twist_msg.linear.x = 0;
                    twist_msg.angular.z = 0;
                    RCLCPP_INFO(this->get_logger(), "Stopping (default)");
            }

            // Publish message
            publisher_command->publish(twist_msg);

        }

};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CBWNode>());
    rclcpp::shutdown();
    return 0;
}



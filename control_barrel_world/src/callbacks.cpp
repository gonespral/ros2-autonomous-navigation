#include "control_barrel_world/control_barrel_world.hpp"

// Define pedestrians topic callback
int CBWNode::pedestriansCallback(const vision_msgs::msg::Detection2DArray::UniquePtr msg) {
    // Store latest pedestrians
    this->latest_pedestrians = msg->detections;
    return 0;
}

int CBWNode::detectionsCallback(const vision_msgs::msg::Detection3DArray::UniquePtr msg) {
    // Store latest detections
    this->latest_detections = msg->detections;

    // Lookup transform from detections frame to base_link first
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
        transform_stamped = this->tf_buffer->lookupTransform(
            "base_link",
            msg->header.frame_id,
            tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
        return 0;
    }

    // Transform each detection position into base_link and store back into latest_detections
    for (auto &detection : latest_detections) {
        geometry_msgs::msg::PointStamped pos_in;
        geometry_msgs::msg::PointStamped pos_out;
        pos_in.header.frame_id = msg->header.frame_id;
        pos_in.header.stamp = msg->header.stamp;
        pos_in.point = detection.bbox.center.position;

        tf2::doTransform(pos_in, pos_out, transform_stamped);

        detection.bbox.center.position = pos_out.point;
    }

    return 0;
}

void CBWNode::timerCallback() {
    // Take most recent detections, process and publish to command topic

    // Prepare new empty message
    auto twist_msg = geometry_msgs::msg::Twist();

    // Action variable 0 = left, 1 = forward, 2 = right, 3 = stop
    int action = 1;
    
    // Filter detections
    std::vector<vision_msgs::msg::Detection2D> latest_pedestrians_proc;
    std::vector<vision_msgs::msg::Detection3D> latest_detections_proc;
    std::vector<double> latest_detections_distances;

    for (const auto& detection : this->latest_detections) {
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
    for (const auto& pedestrian : this->latest_pedestrians) {
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
    this->publisher_command->publish(twist_msg);

}

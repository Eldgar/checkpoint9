#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cmath>

class LegDetection : public rclcpp::Node {
public:
    LegDetection() : Node("leg_detection_node") {
        // Subscriber to laser scan topic
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LegDetection::detect_legs, this, std::placeholders::_1));

        // Subscriber to odometry topic to get the robot's orientation (yaw)
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&LegDetection::odom_callback, this, std::placeholders::_1));

        // Publisher for transforms
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Declare parameters for tuning
        this->declare_parameter("intensity_threshold", 7900.0);
        this->get_parameter("intensity_threshold", intensity_threshold_);

        // Initialize current yaw to 0
        current_yaw_ = 0.0;
    }

private:
    // Odometry callback to track robot's current yaw (orientation)
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double roll, pitch, yaw;
        tf2::Quaternion quat(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        
        // Convert yaw to degrees and store in current_yaw_
        current_yaw_ = yaw * 180.0 / M_PI;
        
        // Normalize yaw to the range [-180, 180]
        if (current_yaw_ > 180.0) {
            current_yaw_ -= 360.0;
        } else if (current_yaw_ < -180.0) {
            current_yaw_ += 360.0;
        }
    }

    // Laser scan callback to detect legs
    void detect_legs(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::vector<double> leg_positions_x;
        std::vector<double> leg_positions_y;

        bool in_cluster = false;
        std::vector<double> cluster_x, cluster_y;
        int low_intensity_counter = 0;

        double min_range = std::numeric_limits<double>::infinity();  // Initialize with infinity
        size_t min_range_index = 0;  // To store the index of the shortest distance

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            double intensity = msg->intensities[i];
            double range = msg->ranges[i];

            // Ignore invalid ranges
            if (std::isinf(range) || std::isnan(range)) {
                continue;
            }

            // Track the minimum range and its index
            if (range < min_range) {
                min_range = range;
                min_range_index = i;
            }

            // Calculate the angle of the current scan point, adjusted by the robot's yaw
            double angle = msg->angle_min + i * msg->angle_increment;
            // Calculate the (x, y) coordinates in the robot's frame
            double x = range * std::cos(angle);
            double y = -range * std::sin(angle);

            if (intensity >= intensity_threshold_) {
                // Add point to current cluster
                in_cluster = true;
                cluster_x.push_back(x);
                cluster_y.push_back(y);
                

                low_intensity_counter = 0;  // reset counter since we're still in a cluster
            } else {
                if (in_cluster) {
                    low_intensity_counter++;
                    if (low_intensity_counter == 1) {
                        // End the current cluster when a low-intensity point is detected
                        in_cluster = false;

                        // Calculate the centroid of the cluster
                        if (!cluster_x.empty() && !cluster_y.empty()) {
                            double cluster_avg_x = std::accumulate(cluster_x.begin(), cluster_x.end(), 0.0) / cluster_x.size();
                            double cluster_avg_y = std::accumulate(cluster_y.begin(), cluster_y.end(), 0.0) / cluster_y.size();
                            RCLCPP_INFO(this->get_logger(), "Current Yaw: %.2f", (current_yaw_* M_PI / 180.0));
                            RCLCPP_INFO(this->get_logger(), "Index: %zu, Range: %.2f, Intensity: %.2f, Angle: %.2f radians (%.2f degrees)", i, range, intensity, angle, angle * 180.0 / M_PI);

                            leg_positions_x.push_back(cluster_avg_x);
                            leg_positions_y.push_back(cluster_avg_y);
                        }

                        // Clear the cluster for the next one
                        cluster_x.clear();
                        cluster_y.clear();

                        // Stop if we found two leg positions
                        if (leg_positions_x.size() >= 2) {
                            break;
                        }
                    }
                }
            }
        }

        // Log the shortest range and its index
        if (!std::isinf(min_range)) {
            //RCLCPP_INFO(this->get_logger(), "Shortest Laser Distance: Range = %.2f at Index = %zu", min_range, min_range_index);
        } else {
            //RCLCPP_WARN(this->get_logger(), "No valid laser data found for shortest distance.");
        }

        // If two legs are detected, publish their transforms and the midpoint
        if (leg_positions_x.size() >= 2) {
            double leg_1_x = leg_positions_x[0];
            double leg_1_y = leg_positions_y[0];
            double leg_2_x = leg_positions_x[1];
            double leg_2_y = leg_positions_y[1];
            double midpoint_x = (leg_1_x + leg_2_x) / 2.0;
            double midpoint_y = (leg_1_y + leg_2_y) / 2.0;

            //RCLCPP_INFO(this->get_logger(), "Legs detected. Publishing transforms for frame_leg_1, frame_leg_2, and cart_frame.");

            // Publish transforms for legs and midpoint
            publish_leg_transform("frame_leg_1", leg_1_x, leg_1_y);
            publish_leg_transform("frame_leg_2", leg_2_x, leg_2_y);
            publish_transform(midpoint_x, midpoint_y);
        } else {
            RCLCPP_WARN(this->get_logger(), "Not enough legs detected.");
        }
    }

    // Function to publish leg transforms
    void publish_leg_transform(const std::string &frame_name, double x, double y) {
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->now();
        transform_stamped.header.frame_id = "robot_base_link";
        transform_stamped.child_frame_id = frame_name;

        transform_stamped.transform.translation.x = x;
        transform_stamped.transform.translation.y = y;
        transform_stamped.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        transform_stamped.transform.rotation = tf2::toMsg(q);

        tf_broadcaster_->sendTransform(transform_stamped);
        //RCLCPP_INFO(this->get_logger(), "Transform for %s published at position (%.2f, %.2f).", frame_name.c_str(), x, y);
    }

    // Function to publish the midpoint transform
    void publish_transform(double x, double y) {
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->now();
        transform_stamped.header.frame_id = "robot_base_link";
        transform_stamped.child_frame_id = "cart_frame";

        transform_stamped.transform.translation.x = x;
        transform_stamped.transform.translation.y = y;
        transform_stamped.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        transform_stamped.transform.rotation = tf2::toMsg(q);

        tf_broadcaster_->sendTransform(transform_stamped);
        RCLCPP_INFO(this->get_logger(), "Transform for cart_frame published at position (%.2f, %.2f).", x, y);
    }

    // Class members
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Parameter for intensity threshold
    double intensity_threshold_;

    // Variable to store the robot's current yaw (in degrees)
    double current_yaw_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LegDetection>());
    rclcpp::shutdown();
    return 0;
}


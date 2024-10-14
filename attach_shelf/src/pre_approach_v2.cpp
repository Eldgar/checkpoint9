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

class PreApproach : public rclcpp::Node {
public:
  PreApproach() : Node("pre_approach") {

    // Declare parameters for obstacle distance and degrees to turn
    this->declare_parameter("obstacle", 0.5);
    this->declare_parameter("degrees", 90);

    // Retrieve the parameters
    this->get_parameters();

    // Publisher for /robot/cmd_vel topic
    vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 40);

    // Subscriber to /scan topic
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 20, std::bind(&PreApproach::scan_callback, this, std::placeholders::_1));

    // Subscriber to /odom topic to get robot's current orientation
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 40, std::bind(&PreApproach::odom_callback, this, std::placeholders::_1));

    // Initialize mode to moving forward
    mode_ = "moving_forward";
    current_angle_ = 0.0;
    target_angle_reached_ = false;

    // Initialize the Transform Broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);


    intensity_threshold_ = 7700.0;
  }

private:
  // Callback function for laser scan
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    geometry_msgs::msg::Twist twist;

    if (mode_ == "moving_forward") {
      // We only care about the range of 10 scan items in front of the robot
      int center_index = msg->ranges.size() / 2;
      float min_range = *std::min_element(msg->ranges.begin() + center_index - 5,
                                          msg->ranges.begin() + center_index + 5);

      if (min_range <= obstacle_distance_) {
        // Stop the robot if it detects an obstacle within the distance
        RCLCPP_INFO(this->get_logger(), "Stopping Robot - Obstacle detected!");

        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
        vel_publisher_->publish(twist);

        // Switch to turning mode
        mode_ = "turning";
        RCLCPP_INFO(this->get_logger(), "Switching to turning mode.");
        target_angle_reached_ = false;
        initial_angle_ = current_angle_;
      } else {
        // Keep moving forward
        twist.linear.x = 0.2;
        twist.angular.z = 0.0;
        vel_publisher_->publish(twist);
      }
    } else if (mode_ == "turning") {
      if (!target_angle_reached_) {
        // Start turning after stopping
        RCLCPP_INFO(this->get_logger(), "Turning the robot.");
        twist.linear.x = 0.0;
        twist.angular.z = turning_speed_;
        vel_publisher_->publish(twist);

        // Check if the robot has turned the desired number of degrees
        if (has_reached_target_angle()) {
          // Stop turning after reaching the target angle
          twist.angular.z = 0.0;
          vel_publisher_->publish(twist);

          // Switch to detecting legs mode
          mode_ = "detecting_legs";
          RCLCPP_INFO(this->get_logger(), "Turn complete. Switching to detecting legs mode.");
        }
      }
    } else if (mode_ == "detecting_legs") {
      // Detect legs using laser intensities
      detect_legs(msg);
    } else if (mode_ == "stopped") {
      // Robot has completed the task and stops
      RCLCPP_INFO(this->get_logger(), "Robot is stopped. No further actions.");
      twist.linear.x = 0.0;
      twist.angular.z = 0.0;
      vel_publisher_->publish(twist);

      rclcpp::shutdown();

    }
  }

    void detect_legs(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::vector<double> leg_positions_x;
        std::vector<double> leg_positions_y;
        std::vector<size_t> cluster_indices;

        bool in_cluster = false;
        std::vector<double> cluster_x, cluster_y;
        int low_intensity_counter = 0;

        for (size_t i = 0; i < msg->intensities.size(); ++i) {
            double intensity = msg->intensities[i];
            double range = msg->ranges[i];

            // Ignore invalid ranges
            if (std::isinf(range) || std::isnan(range)) {
                continue;
            }

            double angle = msg->angle_min + i * msg->angle_increment;
            double x = range * std::cos(angle);
            double y = range * std::sin(angle);

            if (intensity >= intensity_threshold_) {
                // Add point to current cluster
                in_cluster = true;
                cluster_x.push_back(x);
                cluster_y.push_back(y);
                RCLCPP_INFO(this->get_logger(), "Index: %zu, Range: %.2f, Intensity: %.2f", i, range, intensity);

                low_intensity_counter = 0; // reset counter since we're still in a cluster
            } else {
                if (in_cluster) {
                    low_intensity_counter++;
                    if (low_intensity_counter == 1) {
                        // As soon as we detect the first low-intensity point, end the current cluster
                        in_cluster = false;

                        // Calculate cluster centroid (average position)
                        if (!cluster_x.empty() && !cluster_y.empty()) {
                            double cluster_avg_x = std::accumulate(cluster_x.begin(), cluster_x.end(), 0.0) / cluster_x.size();
                            double cluster_avg_y = std::accumulate(cluster_y.begin(), cluster_y.end(), 0.0) / cluster_y.size();

                            leg_positions_x.push_back(cluster_avg_x);
                            leg_positions_y.push_back(cluster_avg_y);

                        }

                        // Clear cluster data for the next one
                        cluster_x.clear();
                        cluster_y.clear();
                        cluster_indices.clear();

                        // Stop if we found 2 leg positions
                        if (leg_positions_x.size() >= 2) {
                            break;
                        }
                    }
                }
            }
        }

        // If we detected 2 clusters (legs), publish transforms for the legs and midpoint
        if (leg_positions_x.size() >= 2) {
            double leg_1_x = leg_positions_x[0];
            double leg_1_y = leg_positions_y[0];
            double leg_2_x = leg_positions_x[1];
            double leg_2_y = leg_positions_y[1];
            double midpoint_x = (leg_1_x + leg_2_x) / 2.0;
            double midpoint_y = (leg_1_y + leg_2_y) / 2.0;

            RCLCPP_INFO(this->get_logger(), "Legs detected. Publishing transforms for frame_leg_1, frame_leg_2, and cart_frame.");

            // Publish transform for leg 1
            publish_leg_transform("frame_leg_1", leg_1_x, leg_1_y);

            // Publish transform for leg 2
            publish_leg_transform("frame_leg_2", leg_2_x, leg_2_y);

            // Publish the transform at the midpoint for cart_frame
            publish_transform(midpoint_x, midpoint_y);

            // Switch to stopped mode
            mode_ = "stopped";
        } else {
            RCLCPP_WARN(this->get_logger(), "Not enough legs detected.");
            // Optionally, you can retry detection or handle failure
        }
    }




  // Function to publish the transform at the midpoint
  void publish_transform(double x, double y) {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = "robot_base_link";
    transform_stamped.child_frame_id = "cart_frame";

    transform_stamped.transform.translation.x = x;
    transform_stamped.transform.translation.y = y;
    transform_stamped.transform.translation.z = 0.0;

    // Set rotation to zero
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transform_stamped.transform.rotation = tf2::toMsg(q);

    // Publish the transform
    tf_broadcaster_->sendTransform(transform_stamped);

    RCLCPP_INFO(this->get_logger(), "Transform published successfully.");
  }

  void publish_leg_transform(const std::string &frame_name, double x, double y) {
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->now();
        transform_stamped.header.frame_id = "robot_base_link"; // Assuming base_link is the robot's parent frame
        transform_stamped.child_frame_id = frame_name;   // Use the passed frame name

        transform_stamped.transform.translation.x = x;
        transform_stamped.transform.translation.y = y;
        transform_stamped.transform.translation.z = 0.0;

        // Set rotation to zero (no rotation)
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        transform_stamped.transform.rotation = tf2::toMsg(q);

        // Publish the transform
        tf_broadcaster_->sendTransform(transform_stamped);

        RCLCPP_INFO(this->get_logger(), "Transform for %s published at position (%.2f, %.2f).", frame_name.c_str(), x, y);
    }


  // Callback for odometry data to track current orientation
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Get the current orientation (yaw) in degrees
    double roll, pitch, yaw;
    tf2::Quaternion quat(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    current_angle_ = yaw * 180.0 / M_PI;

    // Normalize angle to the range [-180, 180]
    if (current_angle_ > 180.0) {
      current_angle_ -= 360.0;
    } else if (current_angle_ < -180.0) {
      current_angle_ += 360.0;
    }
  }

  // Function to check if the robot has reached the target angle
  bool has_reached_target_angle() {
    double angle_diff = (current_angle_ - initial_angle_);

    if (angle_diff > 180.0) {
      angle_diff -= 360.0;
    } else if (angle_diff < -180.0) {
      angle_diff += 360.0;
    }

    // Define tolerance as 0.2% of the target angle
    double tolerance = std::abs(turning_degrees_) * 0.006;

    // Check if the target angle has been reached
    return std::abs(angle_diff) >= std::abs(turning_degrees_) - tolerance &&
           std::abs(angle_diff) <= std::abs(turning_degrees_) + tolerance;
  }

  // Function to convert degrees to radians
  float degrees_to_radians(float degrees) {
    return degrees * M_PI / 180.0;
  }

  void get_parameters() {
    obstacle_distance_ = this->get_parameter("obstacle").as_double();
    turning_degrees_ = this->get_parameter("degrees").as_int();
    turning_speed_ = (turning_degrees_ > 0) ? 0.35 : -0.35;
  }

  std::string mode_;
  double obstacle_distance_;
  double turning_degrees_;
  double current_angle_;
  double initial_angle_;
  double turning_speed_;
  bool target_angle_reached_;

  // Intensity threshold for detecting legs
  double intensity_threshold_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

  // Transform broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreApproach>());
  rclcpp::shutdown();
  return 0;
}

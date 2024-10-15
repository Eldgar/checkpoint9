#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

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
    vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 50);

    // Subscriber to /scan topic
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 30, std::bind(&PreApproach::scan_callback, this, std::placeholders::_1));

    // Subscriber to /odom topic to get robot's current orientation
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 50, std::bind(&PreApproach::odom_callback, this, std::placeholders::_1));

    // Initialize mode to moving forward
    mode_ = "moving_forward";
    current_angle_ = 0.0;
    target_angle_reached_ = false;
  }

private:
  // Callback function for laser scan
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // We only care about the range of 10 scan items in front of the robot
        int center_index = msg->ranges.size() / 2;
        float min_range = *std::min_element(msg->ranges.begin() + center_index - 5,
                                            msg->ranges.begin() + center_index + 5);

        geometry_msgs::msg::Twist twist;

        if (mode_ == "moving_forward") {
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
                    mode_ = "stopped";
                    RCLCPP_INFO(this->get_logger(), "Turn complete. Switching to stopped mode.");
                }
            }
        } else if (mode_ == "stopped") {
            // Robot has completed the task and stops
            RCLCPP_INFO(this->get_logger(), "Robot is stopped. No further actions.");
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            vel_publisher_->publish(twist);

            rclcpp::shutdown();
        }
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
      RCLCPP_INFO(this->get_logger(), "Current angle: %.2f, Initial angle: %.2f, Angle difference: %.2f", 
                current_angle_, initial_angle_, angle_diff);
      RCLCPP_INFO(this->get_logger(), "Target angle: %.2f", 
                turning_degrees_);

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
      turning_speed_ = (turning_degrees_ > 0) ? 0.4 : -0.4;  // Use a fixed turning speed
  }

  std::string mode_;
  double obstacle_distance_;
  double turning_degrees_;
  double current_angle_;
  double initial_angle_;
  double turning_speed_;
  bool target_angle_reached_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreApproach>());
  rclcpp::shutdown();
  return 0;
}


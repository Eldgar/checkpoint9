#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "std_msgs/msg/string.hpp"
#include "attach_shelf/srv/go_to_loading.hpp"

#include <cmath>

class ApproachServiceServer : public rclcpp::Node {
public:
    ApproachServiceServer() : Node("approach_service_server") {
        service_ = this->create_service<attach_shelf::srv::GoToLoading>(
            "/approach_shelf", std::bind(&ApproachServiceServer::handle_service, this, std::placeholders::_1, std::placeholders::_2));


        // Create callback groups for tf, velocity, and a new one for scans, odom, and elevator control
        tf_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        vel_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        scan_odom_elevator_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // Set up subscription options with the scan/odom/elevator callback group
        rclcpp::SubscriptionOptions group_sub_options;
        group_sub_options.callback_group = scan_odom_elevator_group_;

        // Subscriber to laser scan topic
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 50, std::bind(&ApproachServiceServer::detect_legs, this, std::placeholders::_1), group_sub_options);

        // Subscriber to odometry topic to get the robot's orientation (yaw)
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 50, std::bind(&ApproachServiceServer::odom_callback, this, std::placeholders::_1), group_sub_options);


        // Publisher for transforms (associated with the tf callback group)
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Set up publisher options with the velocity callback group
        rclcpp::PublisherOptions vel_pub_options;
        vel_pub_options.callback_group = vel_callback_group_;

        // Publisher for velocity commands
        vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 50, vel_pub_options);

        // Set up publisher options with the scan/odom/elevator callback group
        rclcpp::PublisherOptions elevator_pub_options;
        elevator_pub_options.callback_group = scan_odom_elevator_group_;

        // Publisher for the elevator commands
        elevator_publisher_ = this->create_publisher<std_msgs::msg::String>("/elevator_up", 10, elevator_pub_options);

        // Declare parameters for tuning
        this->declare_parameter("intensity_threshold", 7900.0);
        this->get_parameter("intensity_threshold", intensity_threshold_);

        // Initialize current yaw to 0
        current_yaw_ = 0.0;
    }

private:
    // Service server
    rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr service_;


    bool reached_cart_frame_ = false;
    bool attach_to_shelf_ = false;  // Flag for whether to perform the final approach
 
// Service callback function
    void handle_service(const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
                        std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Received request: attach_to_shelf = %s", request->attach_to_shelf ? "true" : "false");
        attach_to_shelf_ = request->attach_to_shelf;

        // You can perform any initialization here if needed

        response->complete = true;  // Indicate that the request was successfully handled
    }

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
        if (!attach_to_shelf_) {
            return;  // Do nothing if attach_to_shelf is false
        }

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
                            RCLCPP_INFO(this->get_logger(), "Current Yaw: %.2f", (current_yaw_ * M_PI / 180.0));
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

        // If two legs are detected, publish their transforms and the midpoint
        if (leg_positions_x.size() >= 2) {
            double leg_1_x = leg_positions_x[0];
            double leg_1_y = leg_positions_y[0];
            double leg_2_x = leg_positions_x[1];
            double leg_2_y = leg_positions_y[1];
            double midpoint_x = (leg_1_x + leg_2_x) / 2.0;
            double midpoint_y = (leg_1_y + leg_2_y) / 2.0;

            // Publish transforms for legs and midpoint
            publish_transform(midpoint_x, midpoint_y);

            // Move towards the midpoint (cart_frame)
            move_to_transform(midpoint_x, midpoint_y);
        } else {
            RCLCPP_WARN(this->get_logger(), "Not enough legs detected.");
        }
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

    void move_to_transform(double x, double y) {
        geometry_msgs::msg::Twist cmd_vel;
        double tolerance = 0.05;

        // Check if within the tolerance range
        if (std::abs(x) > tolerance || std::abs(y) > tolerance) {
            cmd_vel.linear.x = 0.2;  // Move forward
            cmd_vel.angular.z = 0.0; // No rotation for now
            RCLCPP_INFO(this->get_logger(), "Moving towards the cart_frame (%.2f, %.2f).", x, y);
            vel_publisher_->publish(cmd_vel);
        } else {
            // Robot has reached the cart frame
            RCLCPP_INFO(this->get_logger(), "Reached the cart_frame. Moving forward for 1.5 seconds more.");

            // Keep moving forward for 1.5 seconds after reaching the cart frame
            cmd_vel.linear.x = 0.2;
            cmd_vel.angular.z = 0.0;
            vel_publisher_->publish(cmd_vel);

            // Create a timer to stop after 1.5 seconds
            stop_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(2300), [this]() {
                    geometry_msgs::msg::Twist stop_cmd;
                    stop_cmd.linear.x = 0.0;
                    stop_cmd.angular.z = 0.0;
                    vel_publisher_->publish(stop_cmd);
                    RCLCPP_INFO(this->get_logger(), "Stopped moving after reaching cart_frame.");
                    stop_timer_->cancel();
                });

            stop_timer2_ = this->create_wall_timer(
                std::chrono::milliseconds(2900), [this]() {
                    if (stop_timer_ && !stop_timer_->is_canceled()) {
                        stop_timer_->cancel();
                    }

                    auto elevator_msg = std_msgs::msg::String();
                    elevator_msg.data = "up";
                    elevator_publisher_->publish(elevator_msg);
                    RCLCPP_INFO(this->get_logger(), "Elevator activated to lift the shelf.");
                    stop_timer2_->cancel();
                    rclcpp::shutdown();
                });
        }
    }

    // Class members
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::TimerBase::SharedPtr stop_timer_;
    rclcpp::TimerBase::SharedPtr stop_timer2_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Callback groups for multi-threading
    rclcpp::CallbackGroup::SharedPtr tf_callback_group_;
    rclcpp::CallbackGroup::SharedPtr vel_callback_group_;
    rclcpp::CallbackGroup::SharedPtr scan_odom_elevator_group_;

    // Parameter for intensity threshold
    double intensity_threshold_;

    // Variable to store the robot's current yaw (in degrees)
    double current_yaw_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // Multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor executor;

    // Create the node and add it to the executor
    auto approach_service_server_node = std::make_shared<ApproachServiceServer>();
    executor.add_node(approach_service_server_node);

    // Spin the executor
    executor.spin();
    rclcpp::shutdown();
    return 0;
}


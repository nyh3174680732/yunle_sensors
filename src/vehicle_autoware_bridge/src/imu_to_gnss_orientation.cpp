/**
 * @file imu_to_gnss_orientation.cpp
 * @brief Convert IMU orientation to GnssInsOrientationStamped for gnss_poser
 *
 * This node converts sensor_msgs::msg::Imu to
 * autoware_sensing_msgs::msg::GnssInsOrientationStamped
 *
 * Subscribes: /sensing/imu/tamagawa/imu_raw (sensor_msgs::msg::Imu)
 * Publishes: /autoware_orientation (autoware_sensing_msgs::msg::GnssInsOrientationStamped)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>
#include <cmath>

class ImuToGnssOrientation : public rclcpp::Node
{
public:
    ImuToGnssOrientation() : Node("imu_to_gnss_orientation")
    {
        // Declare parameters
        this->declare_parameter("input_topic", "/sensing/imu/tamagawa/imu_raw");
        this->declare_parameter("output_topic", "/autoware_orientation");
        this->declare_parameter("default_rmse_rotation", 0.1);  // Default RMSE in radians

        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        default_rmse_ = this->get_parameter("default_rmse_rotation").as_double();

        // Subscriber
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            input_topic,
            rclcpp::SensorDataQoS(),
            std::bind(&ImuToGnssOrientation::imuCallback, this, std::placeholders::_1)
        );

        // Publisher
        orientation_pub_ = this->create_publisher<autoware_sensing_msgs::msg::GnssInsOrientationStamped>(
            output_topic, 10);

        RCLCPP_INFO(this->get_logger(),
            "IMU to GNSS Orientation Converter started: %s -> %s",
            input_topic.c_str(), output_topic.c_str());
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        autoware_sensing_msgs::msg::GnssInsOrientationStamped out_msg;

        // Copy header
        out_msg.header = msg->header;

        // Copy orientation quaternion
        out_msg.orientation.orientation = msg->orientation;

        // Set RMSE from IMU covariance or use default
        // IMU orientation_covariance is 3x3 matrix (row-major): [roll, pitch, yaw]
        // covariance[0] = roll variance, covariance[4] = pitch variance, covariance[8] = yaw variance
        if (msg->orientation_covariance[0] > 0) {
            // Convert variance to RMSE (standard deviation)
            out_msg.orientation.rmse_rotation_x = std::sqrt(msg->orientation_covariance[0]);  // Roll
            out_msg.orientation.rmse_rotation_y = std::sqrt(msg->orientation_covariance[4]);  // Pitch
            out_msg.orientation.rmse_rotation_z = std::sqrt(msg->orientation_covariance[8]);  // Yaw
        } else {
            // Use default RMSE if covariance not provided
            out_msg.orientation.rmse_rotation_x = default_rmse_;
            out_msg.orientation.rmse_rotation_y = default_rmse_;
            out_msg.orientation.rmse_rotation_z = default_rmse_ * 2.0;  // Yaw typically has higher uncertainty
        }

        // Publish
        orientation_pub_->publish(out_msg);
    }

    // ROS2 interfaces
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<autoware_sensing_msgs::msg::GnssInsOrientationStamped>::SharedPtr orientation_pub_;

    double default_rmse_{0.1};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuToGnssOrientation>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

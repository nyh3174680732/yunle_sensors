/**
 * @file imu_remapper.cpp
 * @brief Remap IMU topic from /imu/data to /sensing/imu/tamagawa/imu_raw
 *
 * Subscribes: /imu/data (sensor_msgs::msg::Imu)
 * Publishes: /sensing/imu/tamagawa/imu_raw (sensor_msgs::msg::Imu)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class ImuRemapper : public rclcpp::Node
{
public:
    ImuRemapper() : Node("imu_remapper")
    {
        // Declare parameters
        this->declare_parameter("input_topic", "/imu/data");
        this->declare_parameter("output_topic", "/sensing/imu/tamagawa/imu_raw");
        this->declare_parameter("output_frame_id", "tamagawa/imu_link");

        // Get parameters
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        output_frame_id_ = this->get_parameter("output_frame_id").as_string();

        // Subscriber
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            input_topic,
            10,
            std::bind(&ImuRemapper::imuCallback, this, std::placeholders::_1)
        );

        // Publisher
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(output_topic, 10);

        RCLCPP_INFO(this->get_logger(),
            "IMU Remapper started: %s → %s (frame: %s)",
            input_topic.c_str(), output_topic.c_str(), output_frame_id_.c_str());
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Create new message with updated frame_id
        sensor_msgs::msg::Imu output_msg = *msg;
        output_msg.header.frame_id = output_frame_id_;
        output_msg.header.stamp = this->now();

        imu_pub_->publish(output_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    std::string output_frame_id_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuRemapper>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

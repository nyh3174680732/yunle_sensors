/**
 * @file autoware_to_chassis_converter.cpp
 * @brief Convert Autoware control commands to YunLe chassis ECU commands
 *
 * Subscribes: /control/command/control_cmd (autoware_control_msgs::msg::Control)
 * Publishes: /ecu (yunle_msgs::msg::Ecu)
 */

#include <rclcpp/rclcpp.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <yunle_msgs/msg/ecu.hpp>
#include <cmath>

class AutowareToChassisConverter : public rclcpp::Node
{
public:
    AutowareToChassisConverter() : Node("autoware_to_chassis_converter")
    {
        // Declare parameters
        this->declare_parameter("wheel_base", 0.501);  // Vehicle wheelbase in meters
        this->declare_parameter("speed_limit", 5.0);   // Speed limit in m/s
        this->declare_parameter("max_steer_angle", 30.0);  // Max steering angle in degrees
        this->declare_parameter("velocity_threshold", 0.1);  // Threshold for zero velocity (m/s)

        // Get parameters
        wheel_base_ = this->get_parameter("wheel_base").as_double();
        speed_limit_ = this->get_parameter("speed_limit").as_double();
        max_steer_angle_ = this->get_parameter("max_steer_angle").as_double();
        velocity_threshold_ = this->get_parameter("velocity_threshold").as_double();

        // Subscriber to Autoware control command
        control_sub_ = this->create_subscription<autoware_control_msgs::msg::Control>(
            "/control/command/control_cmd",
            10,
            std::bind(&AutowareToChassisConverter::controlCallback, this, std::placeholders::_1)
        );

        // Publisher to chassis ECU
        ecu_pub_ = this->create_publisher<yunle_msgs::msg::Ecu>("chassis/ecu", 10);

        // Watchdog timer - send stop command if no control received for 500ms
        watchdog_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&AutowareToChassisConverter::watchdogCallback, this)
        );

        RCLCPP_INFO(this->get_logger(),
            "Autoware to Chassis Converter started. Wheel base: %.3f m, Speed limit: %.2f m/s",
            wheel_base_, speed_limit_);
    }

private:
    void controlCallback(const autoware_control_msgs::msg::Control::SharedPtr msg)
    {
        last_control_time_ = this->now();

        yunle_msgs::msg::Ecu ecu_msg;
        ecu_msg.header.stamp = this->now();
        ecu_msg.header.frame_id = "base_link";

        // Convert velocity: m/s → km/h
        float velocity_ms = msg->longitudinal.velocity;

        // Apply speed limit
        if (std::abs(velocity_ms) > speed_limit_) {
            velocity_ms = std::copysign(speed_limit_, velocity_ms);
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Speed command %.2f m/s exceeds limit %.2f m/s, clamping",
                msg->longitudinal.velocity, speed_limit_);
        }

        ecu_msg.motor = velocity_ms * 3.6f;  // m/s to km/h

        // Convert steering angle: radians → degrees
        float steer_rad = msg->lateral.steering_tire_angle;
        float steer_deg = steer_rad * 180.0f / M_PI;

        // Apply steering limit
        if (std::abs(steer_deg) > max_steer_angle_) {
            steer_deg = std::copysign(max_steer_angle_, steer_deg);
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Steering command %.2f deg exceeds limit %.2f deg, clamping",
                msg->lateral.steering_tire_angle * 180.0f / M_PI, max_steer_angle_);
        }

        ecu_msg.steer = -steer_deg;

        // Determine gear based on velocity
        if (std::abs(velocity_ms) < velocity_threshold_) {
            ecu_msg.shift = yunle_msgs::msg::Ecu::SHIFT_N;  // Neutral when stopped
        } else if (velocity_ms > 0) {
            ecu_msg.shift = yunle_msgs::msg::Ecu::SHIFT_D;  // Drive forward
        } else {
            ecu_msg.shift = yunle_msgs::msg::Ecu::SHIFT_R;  // Reverse
        }

        // Emergency brake and other flags
        ecu_msg.brake = false;  // Emergency brake controlled separately
        ecu_msg.set_torque = false;
        ecu_msg.rear_wheel_flag = false;

        // Publish ECU command
        ecu_pub_->publish(ecu_msg);

        // Log at lower frequency
        if (std::abs(velocity_ms) > 0.01 || std::abs(steer_deg) > 0.1) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Control → ECU: velocity=%.2f km/h, steer=%.2f deg, shift=%d",
                ecu_msg.motor, ecu_msg.steer, ecu_msg.shift);
        }
    }

    void watchdogCallback()
    {
        auto current_time = this->now();
        auto time_since_last = (current_time - last_control_time_).seconds();

        // If no control command received for more than 500ms, send stop command
        if (time_since_last > 0.5 && last_control_time_.seconds() > 0) {
            yunle_msgs::msg::Ecu stop_msg;
            stop_msg.header.stamp = current_time;
            stop_msg.header.frame_id = "base_link";
            stop_msg.motor = 0.0f;
            stop_msg.steer = 0.0f;
            stop_msg.brake = false;
            stop_msg.shift = yunle_msgs::msg::Ecu::SHIFT_N;
            stop_msg.set_torque = false;
            stop_msg.rear_wheel_flag = false;

            ecu_pub_->publish(stop_msg);

            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "No control command received for %.2f s, sending stop command", time_since_last);
        }
    }

    // ROS2 interfaces
    rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr control_sub_;
    rclcpp::Publisher<yunle_msgs::msg::Ecu>::SharedPtr ecu_pub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    // Parameters
    double wheel_base_;
    double speed_limit_;
    double max_steer_angle_;
    double velocity_threshold_;

    // State
    rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutowareToChassisConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

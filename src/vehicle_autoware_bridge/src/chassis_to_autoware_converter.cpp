/**
 * @file chassis_to_autoware_converter.cpp
 * @brief Convert YunLe chassis status to Autoware vehicle status reports
 *
 * Subscribes: /vehicle_status (yunle_msgs::msg::VehicleStatus)
 * Publishes:
 *   - /vehicle/status/control_mode (ControlModeReport)
 *   - /vehicle/status/gear_status (GearReport)
 *   - /vehicle/status/hazard_lights_status (HazardLightsReport)
 *   - /vehicle/status/steering_status (SteeringReport)
 *   - /vehicle/status/turn_indicators_status (TurnIndicatorsReport)
 *   - /vehicle/status/velocity_status (VelocityReport)
 */

#include <rclcpp/rclcpp.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <yunle_msgs/msg/vehicle_status.hpp>
#include <cmath>

class ChassisToAutowareConverter : public rclcpp::Node
{
public:
    ChassisToAutowareConverter() : Node("chassis_to_autoware_converter")
    {
        // Subscriber to chassis vehicle status
        vehicle_status_sub_ = this->create_subscription<yunle_msgs::msg::VehicleStatus>(
            "chassis/vehicle_status",
            10,
            std::bind(&ChassisToAutowareConverter::vehicleStatusCallback, this, std::placeholders::_1)
        );

        // Publishers to Autoware vehicle status topics
        control_mode_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>(
            "/vehicle/status/control_mode", 10);

        gear_status_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::GearReport>(
            "/vehicle/status/gear_status", 10);

        hazard_lights_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::HazardLightsReport>(
            "/vehicle/status/hazard_lights_status", 10);

        steering_status_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::SteeringReport>(
            "/vehicle/status/steering_status", 10);

        turn_indicators_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>(
            "/vehicle/status/turn_indicators_status", 10);

        velocity_status_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::VelocityReport>(
            "/vehicle/status/velocity_status", 10);

        RCLCPP_INFO(this->get_logger(), "Chassis to Autoware Converter started");
    }

private:
    void vehicleStatusCallback(const yunle_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        auto stamp = this->now();

        // 1. Control Mode Report
        autoware_vehicle_msgs::msg::ControlModeReport control_mode_msg;
        control_mode_msg.stamp = stamp;
        // mode: 1 = AUTONOMOUS, 2 = MANUAL
        control_mode_msg.mode = msg->is_autodrive ?
            autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS :
            autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;  // Always autonomous when connected
        control_mode_pub_->publish(control_mode_msg);

        // 2. Gear Status Report
        autoware_vehicle_msgs::msg::GearReport gear_msg;
        gear_msg.stamp = stamp;
        // Convert YunLe gear to Autoware gear
        // YunLe: 1=D, 2=N, 3=R
        // Autoware: 1=PARK, 2=REVERSE, 20=NEUTRAL, 22=DRIVE, others=LOW/HIGH
        switch (msg->shift_level) {
            case 1:  // Drive 前进
                gear_msg.report = 22;  // DRIVE正常行驶
                break;
            case 2:  // Neutral 停止
                gear_msg.report = 20;  // NEUTRAL 空档
                break;
            case 3:  // Reverse 后退
                gear_msg.report = 2;   // REVERSE 倒档
                break;
            default:
                gear_msg.report = 20;  // NEUTRAL as default
                break;
        }
        gear_status_pub_->publish(gear_msg);

        // 3. Hazard Lights Report (always off for now)
        autoware_vehicle_msgs::msg::HazardLightsReport hazard_msg;
        hazard_msg.stamp = stamp;
        hazard_msg.report = autoware_vehicle_msgs::msg::HazardLightsReport::DISABLE;
        hazard_lights_pub_->publish(hazard_msg);

        // 4. Steering Status Report
        autoware_vehicle_msgs::msg::SteeringReport steering_msg;
        steering_msg.stamp = stamp;
        // Convert degrees to radians
        steering_msg.steering_tire_angle = msg->cur_steer * M_PI / 180.0f;
        steering_status_pub_->publish(steering_msg);

        // 5. Turn Indicators Report (always off for now)
        autoware_vehicle_msgs::msg::TurnIndicatorsReport turn_msg;
        turn_msg.stamp = stamp;
        turn_msg.report = autoware_vehicle_msgs::msg::TurnIndicatorsReport::DISABLE;
        turn_indicators_pub_->publish(turn_msg);

        // 6. Velocity Status Report
        autoware_vehicle_msgs::msg::VelocityReport velocity_msg;
        velocity_msg.header.stamp = stamp;
        velocity_msg.header.frame_id = "base_link";

        // Convert km/h to m/s
        velocity_msg.longitudinal_velocity = msg->cur_speed / 3.6f;
        velocity_msg.lateral_velocity = 0.0f;  // Assume zero lateral velocity

        // Calculate heading rate from wheel speeds (differential)
        // For simple approximation: heading_rate ≈ (v_right - v_left) / wheelbase
        // Note: This is a simplified model, adjust based on actual vehicle kinematics
        float wheel_speed_diff = msg->right_wheel_speed - msg->left_wheel_speed;
        float wheel_base = 0.462f;  // Wheelbase in meters (adjust to actual value)
        velocity_msg.heading_rate = wheel_speed_diff / wheel_base;

        velocity_status_pub_->publish(velocity_msg);

        // Log at lower frequency
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Vehicle Status → Autoware: speed=%.2f m/s, steer=%.2f rad, gear=%d",
            velocity_msg.longitudinal_velocity, steering_msg.steering_tire_angle, gear_msg.report);
    }

    // ROS2 interfaces
    rclcpp::Subscription<yunle_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;

    rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr control_mode_pub_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::GearReport>::SharedPtr gear_status_pub_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::HazardLightsReport>::SharedPtr hazard_lights_pub_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_status_pub_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr turn_indicators_pub_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_status_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChassisToAutowareConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

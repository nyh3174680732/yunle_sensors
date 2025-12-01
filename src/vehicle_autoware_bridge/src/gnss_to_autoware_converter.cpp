/**
 * @file gnss_to_autoware_converter.cpp
 * @brief Convert GNSS NavSatFix to Autoware PoseWithCovarianceStamped
 *
 * This node converts GPS latitude/longitude to local ENU (East-North-Up) coordinates
 * relative to a reference origin point.
 *
 * Subscribes: /sensing/gnss/navsatfix (sensor_msgs::msg::NavSatFix)
 * Publishes: /sensing/gnss/pose_with_covariance (geometry_msgs::msg::PoseWithCovarianceStamped)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <cmath>

class GNSSToAutowareConverter : public rclcpp::Node
{
public:
    GNSSToAutowareConverter() : Node("gnss_to_autoware_converter")
    {
        // Declare parameters
        this->declare_parameter("origin_latitude", 0.0);   // Reference origin latitude
        this->declare_parameter("origin_longitude", 0.0);  // Reference origin longitude
        this->declare_parameter("origin_altitude", 0.0);   // Reference origin altitude
        this->declare_parameter("use_first_fix_as_origin", true);  // Auto-set origin from first fix
        this->declare_parameter("input_topic", "/sensing/gnss/navsatfix");
        this->declare_parameter("output_topic", "/sensing/gnss/pose_with_covariance");

        // Get parameters
        origin_lat_ = this->get_parameter("origin_latitude").as_double();
        origin_lon_ = this->get_parameter("origin_longitude").as_double();
        origin_alt_ = this->get_parameter("origin_altitude").as_double();
        use_first_fix_as_origin_ = this->get_parameter("use_first_fix_as_origin").as_bool();

        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();

        // Check if origin is set
        if (std::abs(origin_lat_) < 1e-6 && std::abs(origin_lon_) < 1e-6) {
            origin_set_ = false;
            if (use_first_fix_as_origin_) {
                RCLCPP_INFO(this->get_logger(), "Will use first GPS fix as origin");
            } else {
                RCLCPP_WARN(this->get_logger(), "Origin not set! Please set origin_latitude and origin_longitude parameters");
            }
        } else {
            origin_set_ = true;
            RCLCPP_INFO(this->get_logger(), "Origin set to: lat=%.8f, lon=%.8f, alt=%.2f",
                origin_lat_, origin_lon_, origin_alt_);
        }

        // Subscriber
        navsat_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            input_topic,
            10,
            std::bind(&GNSSToAutowareConverter::navsatCallback, this, std::placeholders::_1)
        );

        // Publisher
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            output_topic, 10);

        RCLCPP_INFO(this->get_logger(),
            "GNSS to Autoware Converter started: %s → %s",
            input_topic.c_str(), output_topic.c_str());
    }

private:
    void navsatCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        // Set origin from first valid fix if needed
        if (!origin_set_ && use_first_fix_as_origin_) {
            if (msg->status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
                origin_lat_ = msg->latitude;
                origin_lon_ = msg->longitude;
                origin_alt_ = msg->altitude;
                origin_set_ = true;
                RCLCPP_INFO(this->get_logger(),
                    "Origin set from first GPS fix: lat=%.8f, lon=%.8f, alt=%.2f",
                    origin_lat_, origin_lon_, origin_alt_);
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Waiting for valid GPS fix to set origin...");
                return;
            }
        }

        if (!origin_set_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Origin not set, cannot convert GNSS to pose");
            return;
        }

        // Convert lat/lon to local ENU coordinates
        double east, north, up;
        wgs84ToENU(msg->latitude, msg->longitude, msg->altitude,
                   origin_lat_, origin_lon_, origin_alt_,
                   east, north, up);

        // Create PoseWithCovarianceStamped message
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header = msg->header;
        pose_msg.header.frame_id = "map";  // Use "map" frame for global reference

        // Set position (ENU coordinates)
        pose_msg.pose.pose.position.x = east;
        pose_msg.pose.pose.position.y = north;
        pose_msg.pose.pose.position.z = up;

        // Set orientation (identity quaternion - no heading info from GPS alone)
        pose_msg.pose.pose.orientation.w = 1.0;
        pose_msg.pose.pose.orientation.x = 0.0;
        pose_msg.pose.pose.orientation.y = 0.0;
        pose_msg.pose.pose.orientation.z = 0.0;

        // Set covariance (6x6 matrix for position and orientation)
        // Copy position covariance from NavSatFix
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                pose_msg.pose.covariance[i * 6 + j] = msg->position_covariance[i * 3 + j];
            }
        }

        // Set high uncertainty for orientation (since we don't have heading)
        pose_msg.pose.covariance[21] = 1.0;  // Roll variance
        pose_msg.pose.covariance[28] = 1.0;  // Pitch variance
        pose_msg.pose.covariance[35] = 10.0; // Yaw variance (high uncertainty)

        // Publish
        pose_pub_->publish(pose_msg);

        // Log at lower frequency
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "GNSS → Pose: E=%.2f, N=%.2f, U=%.2f (from lat=%.8f, lon=%.8f)",
            east, north, up, msg->latitude, msg->longitude);
    }

    /**
     * @brief Convert WGS84 coordinates to local ENU (East-North-Up)
     *
     * This is a simplified flat-Earth approximation suitable for small areas.
     * For large areas, use proper geodetic transformations.
     */
    void wgs84ToENU(double lat, double lon, double alt,
                    double lat0, double lon0, double alt0,
                    double& east, double& north, double& up)
    {
        // WGS84 semi-major axis
        const double a = 6378137.0;  // meters
        const double e2 = 0.00669437999014;  // First eccentricity squared

        // Convert degrees to radians
        double lat_rad = lat * M_PI / 180.0;
        double lon_rad = lon * M_PI / 180.0;
        double lat0_rad = lat0 * M_PI / 180.0;
        double lon0_rad = lon0 * M_PI / 180.0;

        // Compute radius of curvature in the prime vertical
        double N = a / std::sqrt(1.0 - e2 * std::sin(lat0_rad) * std::sin(lat0_rad));

        // Compute ENU coordinates (flat-Earth approximation)
        double dLat = lat_rad - lat0_rad;
        double dLon = lon_rad - lon0_rad;

        east = (N + alt0) * std::cos(lat0_rad) * dLon;
        north = (N * (1.0 - e2) + alt0) * dLat;
        up = alt - alt0;
    }

    // ROS2 interfaces
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;

    // Origin parameters
    double origin_lat_{0.0};
    double origin_lon_{0.0};
    double origin_alt_{0.0};
    bool origin_set_{false};
    bool use_first_fix_as_origin_{true};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GNSSToAutowareConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

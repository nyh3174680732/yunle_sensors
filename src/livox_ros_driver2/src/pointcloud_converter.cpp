//
// Livox PointCloud2 Extended Format Converter
// Converts Livox MID360 pointcloud format to extended format with azimuth/elevation/distance
//

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cstring>
#include <cmath>

class PointCloudConverter : public rclcpp::Node
{
public:
  PointCloudConverter() : Node("livox_pointcloud_converter")
  {
    // Declare parameters
    this->declare_parameter<std::string>("input_topic", "/livox/lidar");
    this->declare_parameter<std::string>("output_topic", "/sensing/lidar/top/pointcloud_raw_ex");
    this->declare_parameter<std::string>("output_frame_id", "velodyne_top_base_link");

    // Get parameters
    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    output_frame_id_ = this->get_parameter("output_frame_id").as_string();

    // Create subscriber and publisher
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, 10,
      std::bind(&PointCloudConverter::pointcloud_callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);

    RCLCPP_INFO(this->get_logger(), "PointCloud Converter initialized");
    RCLCPP_INFO(this->get_logger(), "  Input topic:  %s", input_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Output topic: %s", output_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Output frame: %s", output_frame_id_.c_str());
  }

private:
  // Livox point structure (26 bytes)
  struct LivoxPoint
  {
    float x;           // 4 bytes
    float y;           // 4 bytes
    float z;           // 4 bytes
    float intensity;   // 4 bytes
    uint8_t tag;       // 1 byte
    uint8_t line;      // 1 byte
    double timestamp;  // 8 bytes
  } __attribute__((packed));

  // Extended point structure (32 bytes)
  struct ExtendedPoint
  {
    float x;              // 4 bytes, offset: 0
    float y;              // 4 bytes, offset: 4
    float z;              // 4 bytes, offset: 8
    uint8_t intensity;    // 1 byte,  offset: 12
    uint8_t return_type;  // 1 byte,  offset: 13
    uint16_t channel;     // 2 bytes, offset: 14
    float azimuth;        // 4 bytes, offset: 16
    float elevation;      // 4 bytes, offset: 20
    float distance;       // 4 bytes, offset: 24
    uint32_t time_stamp;  // 4 bytes, offset: 28
  } __attribute__((packed));

  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Create output message
    sensor_msgs::msg::PointCloud2 output_msg;
    output_msg.header.stamp = msg->header.stamp;
    output_msg.header.frame_id = output_frame_id_;
    output_msg.height = 1;
    output_msg.width = msg->width;
    output_msg.is_bigendian = false;
    output_msg.is_dense = true;

    // Define point fields for extended format (32 bytes)
    output_msg.fields.resize(10);

    // x - FLOAT32
    output_msg.fields[0].name = "x";
    output_msg.fields[0].offset = 0;
    output_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    output_msg.fields[0].count = 1;

    // y - FLOAT32
    output_msg.fields[1].name = "y";
    output_msg.fields[1].offset = 4;
    output_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    output_msg.fields[1].count = 1;

    // z - FLOAT32
    output_msg.fields[2].name = "z";
    output_msg.fields[2].offset = 8;
    output_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    output_msg.fields[2].count = 1;

    // intensity - UINT8
    output_msg.fields[3].name = "intensity";
    output_msg.fields[3].offset = 12;
    output_msg.fields[3].datatype = sensor_msgs::msg::PointField::UINT8;
    output_msg.fields[3].count = 1;

    // return_type - UINT8
    output_msg.fields[4].name = "return_type";
    output_msg.fields[4].offset = 13;
    output_msg.fields[4].datatype = sensor_msgs::msg::PointField::UINT8;
    output_msg.fields[4].count = 1;

    // channel - UINT16
    output_msg.fields[5].name = "channel";
    output_msg.fields[5].offset = 14;
    output_msg.fields[5].datatype = sensor_msgs::msg::PointField::UINT16;
    output_msg.fields[5].count = 1;

    // azimuth - FLOAT32
    output_msg.fields[6].name = "azimuth";
    output_msg.fields[6].offset = 16;
    output_msg.fields[6].datatype = sensor_msgs::msg::PointField::FLOAT32;
    output_msg.fields[6].count = 1;

    // elevation - FLOAT32
    output_msg.fields[7].name = "elevation";
    output_msg.fields[7].offset = 20;
    output_msg.fields[7].datatype = sensor_msgs::msg::PointField::FLOAT32;
    output_msg.fields[7].count = 1;

    // distance - FLOAT32
    output_msg.fields[8].name = "distance";
    output_msg.fields[8].offset = 24;
    output_msg.fields[8].datatype = sensor_msgs::msg::PointField::FLOAT32;
    output_msg.fields[8].count = 1;

    // time_stamp - UINT32
    output_msg.fields[9].name = "time_stamp";
    output_msg.fields[9].offset = 28;
    output_msg.fields[9].datatype = sensor_msgs::msg::PointField::UINT32;
    output_msg.fields[9].count = 1;

    output_msg.point_step = 32;
    output_msg.row_step = output_msg.point_step * output_msg.width;
    output_msg.data.resize(output_msg.row_step);

    // Parse input Livox pointcloud
    const uint8_t* input_data = msg->data.data();
    uint8_t* output_data = output_msg.data.data();

    // Find base timestamp for relative time calculation
    uint64_t base_timestamp = 0;
    bool has_timestamp = false;

    // Check if the input has the timestamp field
    for (const auto& field : msg->fields) {
      if (field.name == "timestamp") {
        has_timestamp = true;
        break;
      }
    }

    // Get base timestamp from first point if available
    if (has_timestamp && msg->width > 0) {
      const LivoxPoint* first_point = reinterpret_cast<const LivoxPoint*>(input_data);
      base_timestamp = static_cast<uint64_t>(first_point->timestamp);
    }

    // Convert each point
    for (size_t i = 0; i < msg->width; ++i)
    {
      const LivoxPoint* livox_point = reinterpret_cast<const LivoxPoint*>(
        input_data + i * msg->point_step);
      ExtendedPoint* ext_point = reinterpret_cast<ExtendedPoint*>(
        output_data + i * output_msg.point_step);

      // Copy x, y, z
      ext_point->x = livox_point->x;
      ext_point->y = livox_point->y;
      ext_point->z = livox_point->z;

      // Convert intensity from float to uint8 (0-255 range)
      float intensity_value = livox_point->intensity;
      if (intensity_value > 255.0f) intensity_value = 255.0f;
      if (intensity_value < 0.0f) intensity_value = 0.0f;
      ext_point->intensity = static_cast<uint8_t>(intensity_value);

      // Map tag to return_type
      ext_point->return_type = livox_point->tag;

      // Map line to channel
      ext_point->channel = static_cast<uint16_t>(livox_point->line);

      // Calculate spherical coordinates
      float x = livox_point->x;
      float y = livox_point->y;
      float z = livox_point->z;

      // Distance (range)
      ext_point->distance = std::sqrt(x * x + y * y + z * z);

      // Azimuth (horizontal angle in radians, from +X axis, counter-clockwise)
      ext_point->azimuth = std::atan2(y, x);

      // Elevation (vertical angle in radians, from XY plane)
      float xy_dist = std::sqrt(x * x + y * y);
      ext_point->elevation = std::atan2(z, xy_dist);

      // Convert timestamp to relative time in nanoseconds
      if (has_timestamp) {
        uint64_t point_timestamp = static_cast<uint64_t>(livox_point->timestamp);
        ext_point->time_stamp = static_cast<uint32_t>(point_timestamp - base_timestamp);
      } else {
        ext_point->time_stamp = 0;
      }
    }

    // Publish converted pointcloud
    pub_->publish(output_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  std::string output_frame_id_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudConverter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

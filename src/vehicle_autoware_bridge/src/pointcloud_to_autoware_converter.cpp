//
// PointCloud2 Extended Format Converter
// Converts pointcloud formats to Autoware extended format with azimuth/elevation/distance
// Supports: Livox MID360 (top), Single-line lidars (left/right)
//

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cstring>
#include <cmath>

class PointCloudConverter : public rclcpp::Node
{
public:
  PointCloudConverter() : Node("pointcloud_to_autoware_converter")
  {
    // Declare parameters
    this->declare_parameter<std::string>("livox_input_topic", "/livox/lidar");
    this->declare_parameter<std::string>("top_output_topic", "/sensing/lidar/top/pointcloud_raw_ex");
    this->declare_parameter<std::string>("top_frame_id", "velodyne_top_base_link");

    // 单线雷达参数
    this->declare_parameter<std::string>("front_input_topic", "laser/cloud_front");
    this->declare_parameter<std::string>("left_output_topic", "/sensing/lidar/left/pointcloud_raw_ex");
    this->declare_parameter<std::string>("left_frame_id", "velodyne_left_base_link");

    this->declare_parameter<std::string>("back_input_topic", "laser/cloud_back");
    this->declare_parameter<std::string>("right_output_topic", "/sensing/lidar/right/pointcloud_raw_ex");
    this->declare_parameter<std::string>("right_frame_id", "velodyne_right_base_link");

    // 使用系统时间而不是传感器时间
    this->declare_parameter<bool>("use_system_time", true);

    // Get parameters
    std::string livox_input_topic = this->get_parameter("livox_input_topic").as_string();
    std::string top_output_topic = this->get_parameter("top_output_topic").as_string();
    top_frame_id_ = this->get_parameter("top_frame_id").as_string();

    std::string front_input_topic = this->get_parameter("front_input_topic").as_string();
    std::string left_output_topic = this->get_parameter("left_output_topic").as_string();
    left_frame_id_ = this->get_parameter("left_frame_id").as_string();

    std::string back_input_topic = this->get_parameter("back_input_topic").as_string();
    std::string right_output_topic = this->get_parameter("right_output_topic").as_string();
    right_frame_id_ = this->get_parameter("right_frame_id").as_string();

    use_system_time_ = this->get_parameter("use_system_time").as_bool();

    // 使用可靠传输 QoS (RELIABLE)
    auto sensor_qos = rclcpp::QoS(10).reliable();

    // Create subscribers with RELIABLE QoS
    // Livox MID360 subscriber
    sub_livox_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      livox_input_topic, sensor_qos,
      std::bind(&PointCloudConverter::livox_callback, this, std::placeholders::_1));

    // Front single-line lidar subscriber (-> left)
    sub_front_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      front_input_topic, sensor_qos,
      std::bind(&PointCloudConverter::front_callback, this, std::placeholders::_1));

    // Back single-line lidar subscriber (-> right)
    sub_back_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      back_input_topic, sensor_qos,
      std::bind(&PointCloudConverter::back_callback, this, std::placeholders::_1));

    // Create publishers with RELIABLE QoS
    pub_top_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      top_output_topic, sensor_qos);
    pub_left_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      left_output_topic, sensor_qos);
    pub_right_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      right_output_topic, sensor_qos);

    RCLCPP_INFO(this->get_logger(), "PointCloud Converter initialized with RELIABLE QoS");
    RCLCPP_INFO(this->get_logger(), "  Livox input:  %s -> %s (frame: %s)",
                livox_input_topic.c_str(), top_output_topic.c_str(), top_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Front input:  %s -> %s (frame: %s)",
                front_input_topic.c_str(), left_output_topic.c_str(), left_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Back input:   %s -> %s (frame: %s)",
                back_input_topic.c_str(), right_output_topic.c_str(), right_frame_id_.c_str());
  }
  rclcpp::Time last_livox_time_;

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

  // Standard point structure from laserscan_to_pointcloud (16 bytes typically)
  struct StandardPoint
  {
    float x;           // 4 bytes
    float y;           // 4 bytes
    float z;           // 4 bytes
    float intensity;   // 4 bytes
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

  // Setup extended point fields
  void setup_extended_fields(sensor_msgs::msg::PointCloud2& msg)
  {
    msg.fields.resize(10);

    // x - FLOAT32
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[0].count = 1;

    // y - FLOAT32
    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[1].count = 1;

    // z - FLOAT32
    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[2].count = 1;

    // intensity - UINT8
    msg.fields[3].name = "intensity";
    msg.fields[3].offset = 12;
    msg.fields[3].datatype = sensor_msgs::msg::PointField::UINT8;
    msg.fields[3].count = 1;

    // return_type - UINT8
    msg.fields[4].name = "return_type";
    msg.fields[4].offset = 13;
    msg.fields[4].datatype = sensor_msgs::msg::PointField::UINT8;
    msg.fields[4].count = 1;

    // channel - UINT16
    msg.fields[5].name = "channel";
    msg.fields[5].offset = 14;
    msg.fields[5].datatype = sensor_msgs::msg::PointField::UINT16;
    msg.fields[5].count = 1;

    // azimuth - FLOAT32
    msg.fields[6].name = "azimuth";
    msg.fields[6].offset = 16;
    msg.fields[6].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[6].count = 1;

    // elevation - FLOAT32
    msg.fields[7].name = "elevation";
    msg.fields[7].offset = 20;
    msg.fields[7].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[7].count = 1;

    // distance - FLOAT32
    msg.fields[8].name = "distance";
    msg.fields[8].offset = 24;
    msg.fields[8].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[8].count = 1;

    // time_stamp - UINT32
    msg.fields[9].name = "time_stamp";
    msg.fields[9].offset = 28;
    msg.fields[9].datatype = sensor_msgs::msg::PointField::UINT32;
    msg.fields[9].count = 1;

    msg.point_step = 32;
  }

  // Convert standard pointcloud to extended format
  void convert_standard_to_extended(
    const sensor_msgs::msg::PointCloud2::SharedPtr input_msg,
    sensor_msgs::msg::PointCloud2& output_msg,
    const std::string& frame_id)
  {
    // Setup header
    if (use_system_time_) {
      //output_msg.header.stamp = this->now();
      output_msg.header.stamp = last_livox_time_;
    } else {
      output_msg.header.stamp = input_msg->header.stamp;
    }
    output_msg.header.frame_id = frame_id;
    output_msg.height = 1;
    output_msg.is_bigendian = false;
    output_msg.is_dense = true;

    // Setup fields
    setup_extended_fields(output_msg);

    // Find field offsets in input message
    int x_offset = -1, y_offset = -1, z_offset = -1, intensity_offset = -1;
    for (const auto& field : input_msg->fields) {
      if (field.name == "x") x_offset = field.offset;
      else if (field.name == "y") y_offset = field.offset;
      else if (field.name == "z") z_offset = field.offset;
      else if (field.name == "intensity") intensity_offset = field.offset;
    }

    if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Input pointcloud missing required fields (x, y, z)");
      return;
    }

    // Count valid points first
    std::vector<ExtendedPoint> valid_points;
    valid_points.reserve(input_msg->width * input_msg->height);

    const uint8_t* input_data = input_msg->data.data();
    size_t total_points = input_msg->width * input_msg->height;

    for (size_t i = 0; i < total_points; ++i) {
      const uint8_t* point_ptr = input_data + i * input_msg->point_step;

      float x, y, z;
      std::memcpy(&x, point_ptr + x_offset, sizeof(float));
      std::memcpy(&y, point_ptr + y_offset, sizeof(float));
      std::memcpy(&z, point_ptr + z_offset, sizeof(float));

      // Skip invalid points (NaN or inf)
      if (std::isnan(x) || std::isnan(y) || std::isnan(z) ||
          std::isinf(x) || std::isinf(y) || std::isinf(z)) {
        continue;
      }

      ExtendedPoint ext_point;
      ext_point.x = x;
      ext_point.y = y;
      ext_point.z = z;

      // Get intensity if available
      float intensity = 0.0f;
      if (intensity_offset >= 0) {
        std::memcpy(&intensity, point_ptr + intensity_offset, sizeof(float));
      }

      // Convert intensity to uint8
      if (std::isnan(intensity) || intensity < 0.0f) intensity = 0.0f;
      if (intensity > 255.0f) intensity = 255.0f;
      ext_point.intensity = static_cast<uint8_t>(intensity);

      // Default values for single-line lidar
      ext_point.return_type = 0;
      ext_point.channel = 0;

      // Calculate spherical coordinates
      ext_point.distance = std::sqrt(x * x + y * y + z * z);
      ext_point.azimuth = std::atan2(y, x);
      float xy_dist = std::sqrt(x * x + y * y);
      ext_point.elevation = std::atan2(z, xy_dist);
      ext_point.time_stamp = 0;

      valid_points.push_back(ext_point);
    }

    // Setup output message
    output_msg.width = valid_points.size();
    output_msg.row_step = output_msg.point_step * output_msg.width;
    output_msg.data.resize(output_msg.row_step);

    // Copy valid points to output
    std::memcpy(output_msg.data.data(), valid_points.data(),
                valid_points.size() * sizeof(ExtendedPoint));
  }

  // Livox MID360 callback
  void livox_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    sensor_msgs::msg::PointCloud2 output_msg;

    // Setup header
    if (use_system_time_) {
      last_livox_time_ = this->now(); 
      output_msg.header.stamp = last_livox_time_;
    } else {
      output_msg.header.stamp = msg->header.stamp;
    }
    output_msg.header.frame_id = top_frame_id_;
    output_msg.height = 1;
    output_msg.width = msg->width;
    output_msg.is_bigendian = false;
    output_msg.is_dense = true;

    // Setup fields
    setup_extended_fields(output_msg);
    output_msg.row_step = output_msg.point_step * output_msg.width;
    output_msg.data.resize(output_msg.row_step);

    // Parse input Livox pointcloud
    const uint8_t* input_data = msg->data.data();
    uint8_t* output_data = output_msg.data.data();

    // Check if the input has the timestamp field
    bool has_timestamp = false;
    for (const auto& field : msg->fields) {
      if (field.name == "timestamp") {
        has_timestamp = true;
        break;
      }
    }

    // Get base timestamp from first point if available
    uint64_t base_timestamp = 0;
    if (has_timestamp && msg->width > 0) {
      const LivoxPoint* first_point = reinterpret_cast<const LivoxPoint*>(input_data);
      base_timestamp = static_cast<uint64_t>(first_point->timestamp);
    }

    // Convert each point
    for (size_t i = 0; i < msg->width; ++i) {
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

      ext_point->distance = std::sqrt(x * x + y * y + z * z);
      ext_point->azimuth = std::atan2(y, x);
      float xy_dist = std::sqrt(x * x + y * y);
      ext_point->elevation = std::atan2(z, xy_dist);

      // Convert timestamp to relative time
      if (has_timestamp) {
        uint64_t point_timestamp = static_cast<uint64_t>(livox_point->timestamp);
        ext_point->time_stamp = static_cast<uint32_t>(point_timestamp - base_timestamp);
      } else {
        ext_point->time_stamp = 0;
      }
    }

    pub_top_->publish(output_msg);
  }

  // Front single-line lidar callback (-> left)
  void front_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    sensor_msgs::msg::PointCloud2 output_msg;
    convert_standard_to_extended(msg, output_msg, left_frame_id_);
    if (output_msg.width > 0) {
      pub_left_->publish(output_msg);
    }
  }

  // Back single-line lidar callback (-> right)
  void back_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    sensor_msgs::msg::PointCloud2 output_msg;
    convert_standard_to_extended(msg, output_msg, right_frame_id_);
    if (output_msg.width > 0) {
      pub_right_->publish(output_msg);
    }
  }

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_livox_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_front_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_back_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_top_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_left_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_right_;

  // Frame IDs
  std::string top_frame_id_;
  std::string left_frame_id_;
  std::string right_frame_id_;

  bool use_system_time_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudConverter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

# Sensor Bringup Package

这个包提供了用于启动所有传感器的集成 launch 文件。

## 功能

同时启动以下传感器：
- **Livox MID360 激光雷达** - 3D 点云数据
- **点云格式转换器** - 将 Livox 格式转换为扩展格式（32 bytes/点）
- **IMU/AHRS 传感器** - 姿态和加速度数据
- **GPS 接收器** - 定位数据

## 使用方法

### 1. 编译包

```bash
cd ~/Desktop/yunle_sensors
colcon build --packages-select sensor_bringup
source install/setup.bash
```

### 2. 使用默认参数启动所有传感器

```bash
ros2 launch sensor_bringup all_sensors_launch.py
```

### 3. 自定义串口参数启动

```bash
ros2 launch sensor_bringup all_sensors_launch.py \
  gps_serial_port:=/dev/ttyACM0 \
  gps_baudrate:=115200 \
  imu_serial_port:=/dev/ttyUSB0 \
  imu_baudrate:=921600
```

## 发布的 Topics

### 激光雷达
- `/livox/lidar` - 原始 Livox 点云数据 (sensor_msgs/PointCloud2, 26 bytes/点)
- `/sensing/lidar/top/pointcloud_raw_ex` - 扩展格式点云 (sensor_msgs/PointCloud2, 32 bytes/点)
  - 包含字段: x, y, z, intensity, return_type, channel, azimuth, elevation, distance, time_stamp
- `/livox/imu` - Livox 内置 IMU 数据 (sensor_msgs/Imu)

### IMU/AHRS
- `/imu/data` - IMU 数据 (sensor_msgs/Imu)
- `/imu/euler_angles` - 欧拉角
- `/imu/magnetic` - 磁场数据
- `/imu/mag_pose_2d` - 2D 磁航向
- `/imu/twist` - 速度数据
- `/imu/ned_odometry` - NED 坐标系里程计

### GPS
- `/sensing/gnss/pose_with_covariance` - GPS 定位数据 (sensor_msgs/NavSatFix)
  - 包含纬度、经度、海拔高度
  - 包含定位质量状态（无定位、GPS、DGPS、RTK等）
  - 包含基于 HDOP 的位置协方差

## 启动参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| gps_serial_port | /dev/ttyACM0 | GPS 接收器串口设备 |
| gps_baudrate | 115200 | GPS 串口波特率 |
| imu_serial_port | /dev/ttyUSB0 | IMU 传感器串口设备 |
| imu_baudrate | 921600 | IMU 串口波特率 |

## 依赖包

- `livox_ros_driver2` - Livox 激光雷达驱动
- `fdilink_ahrs` - IMU/AHRS 驱动
- `gnss_converter` - GPS 转换器

## 故障排查

### 串口权限问题

```bash
# 添加用户到 dialout 组（需要重新登录生效）
sudo usermod -a -G dialout $USER

# 或临时赋予权限
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyACM0
```

### 查找串口设备

```bash
# 列出所有串口设备
ls -l /dev/tty*

# 实时查看设备连接
dmesg | grep -i tty

# 查看 USB 设备
lsusb
```

### 检查 topic 发布

```bash
# 查看所有 topics
ros2 topic list

# 查看点云数据
ros2 topic echo /sensing/lidar/top/pointcloud_raw_ex --once

# 查看 IMU 数据
ros2 topic echo /imu/data --once

# 查看 GPS 数据
ros2 topic echo /sensing/gnss/pose_with_covariance --once

# 查看 topic 频率
ros2 topic hz /sensing/lidar/top/pointcloud_raw_ex
```

### Livox 网络配置

确保计算机与 Livox 雷达在同一网段：

```bash
# 检查网络连接
ping 192.168.1.187

# 配置静态 IP（如果需要）
sudo ip addr add 192.168.1.50/24 dev eth0
```

## 可视化

使用 RViz2 可视化传感器数据：

```bash
# 启动传感器
ros2 launch sensor_bringup all_sensors_launch.py

# 在另一个终端启动 RViz
ros2 run rviz2 rviz2
```

在 RViz 中添加：
- **PointCloud2** - 显示 `/sensing/lidar/top/pointcloud_raw_ex`
- **TF** - 显示坐标系变换
- **Imu** - 显示 `/imu/data`
- **NavSatFix** - 显示 `/sensing/gnss/pose_with_covariance` (GPS位置)

## 注意事项

1. **启动顺序**: 所有节点同时启动，确保所有硬件已连接并上电
2. **网络配置**: Livox 雷达需要配置静态 IP (默认 192.168.1.1XX)
3. **串口冲突**: 确保 GPS 和 IMU 使用不同的串口设备
4. **权限问题**: 首次使用需要配置串口访问权限

## License

MIT

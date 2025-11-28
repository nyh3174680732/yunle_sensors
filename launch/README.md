# 传感器统一启动文件说明

## 文件说明

`all_sensors_launch.py` - 统一启动激光雷达、IMU和GPS转换的启动文件

## 使用方法

### 1. 基本启动（使用默认参数）

```bash
cd /home/cplus/Desktop/yunle_sensors
ros2 launch launch/all_sensors_launch.py
```

### 2. 自定义GPS串口参数启动

```bash
ros2 launch launch/all_sensors_launch.py gps_serial_port:=/dev/ttyUSB1 gps_baudrate:=9600
```

### 3. 完整参数启动

```bash
ros2 launch launch/all_sensors_launch.py \
    gps_serial_port:=/dev/ttyACM0 \
    gps_baudrate:=115200 \
    gps_frame_id:=gnss_link
```

## 默认配置

### 激光雷达 (Livox MID360)
- **数据格式**: 自定义点云格式
- **发布频率**: 10 Hz
- **坐标系**: livox_frame
- **配置文件**: src/livox_ros_driver2/config/MID360_config.json

### IMU/AHRS (FDiLink)
- **串口**: /dev/ttyUSB0
- **波特率**: 921600
- **坐标系**: gyro_link
- **发布话题**:
  - /imu
  - /mag_pose_2d
  - /magnetic
  - /euler_angles
  - /NED_odometry

### GPS转换 (GNSS Converter)
- **串口**: /dev/ttyACM0 (可通过参数修改)
- **波特率**: 115200 (可通过参数修改)
- **坐标系**: gnss_link (可通过参数修改)

## 注意事项

1. **串口权限**: 确保当前用户有串口设备的读写权限
   ```bash
   sudo chmod 666 /dev/ttyUSB0
   sudo chmod 666 /dev/ttyACM0
   ```
   或者将用户添加到dialout组：
   ```bash
   sudo usermod -a -G dialout $USER
   # 需要重新登录才能生效
   ```

2. **设备连接**: 启动前请确保所有传感器已正确连接
   - Livox激光雷达（网络连接）
   - IMU设备（通常是/dev/ttyUSB0）
   - GPS设备（通常是/dev/ttyACM0）

3. **配置文件**: 如需修改激光雷达参数，请编辑：
   `src/livox_ros_driver2/config/MID360_config.json`

4. **查看设备**: 检查串口设备
   ```bash
   ls /dev/tty*
   ```

## 故障排除

### 问题1: 找不到串口设备
```bash
# 查看所有串口设备
ls -l /dev/tty*

# 查看USB设备
lsusb
```

### 问题2: 权限不足
```bash
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyACM0
```

### 问题3: 激光雷达无法连接
- 检查网络连接
- 检查IP配置
- 查看配置文件中的IP地址设置

## 查看发布的话题

```bash
# 查看所有话题
ros2 topic list

# 查看IMU数据
ros2 topic echo /imu

# 查看激光雷达数据
ros2 topic echo /livox/lidar

# 查看GPS数据
ros2 topic echo /gnss_converter/pose
```

## 可视化

使用RViz2可视化传感器数据：
```bash
rviz2
```

在RViz2中添加相应的显示项：
- PointCloud2 - 激光雷达点云
- TF - 坐标变换
- Pose - GPS位置

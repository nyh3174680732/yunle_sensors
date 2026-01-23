# Vehicle-Autoware Bridge - 快速入门

## 1. 编译

```bash
cd ~/yunle_sensors  # 或你的工作空间
colcon build --packages-select vehicle_autoware_bridge
source install/setup.bash
```

## 2. 快速测试

### 步骤 1: 启动传感器和底盘
```bash
# 终端 1: 启动传感器
ros2 launch sensor_bringup all_sensors_launch.py

# 终端 2: 启动底盘驱动 (如有)
ros2 launch chassis_driver chassis_driver.launch.py
```

### 步骤 2: 启动桥接节点
```bash
# 终端 3: 启动桥接
ros2 launch vehicle_autoware_bridge vehicle_autoware_bridge.launch.py
```

### 步骤 3: 检查话题
```bash
# 检查所有话题
ros2 topic list | grep -E "(sensing|vehicle|control)"

# 应该看到以下话题:
# /sensing/lidar/top/pointcloud_raw_ex
# /sensing/imu/tamagawa/imu_raw
# /sensing/gnss/pose_with_covariance
# /vehicle/status/velocity_status
# /vehicle/status/steering_status
# /vehicle/status/gear_status
# /control/command/control_cmd
```

### 步骤 4: 测试控制 (小心!)
```bash
# 发送测试控制命令 (速度 0.5 m/s)
ros2 topic pub /control/command/control_cmd autoware_control_msgs/msg/Control "{
  longitudinal: {velocity: 0.5, acceleration: 0.0},
  lateral: {steering_tire_angle: 0.0}
}" --once
```

## 3. 与 Autoware 集成

启动完整系统后，在 Autoware 端：

```bash
# 在 Autoware 工作空间
ros2 launch autoware_launch autoware.launch.xml \
  map_path:=/path/to/your/map \
  vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit
```

## 4. 常见问题

### Q: 车辆不响应控制命令
A: 检查 `/ecu` 话题是否有数据发布

### Q: GPS 坐标不准
A: 等待 GPS 定位稳定后重启节点，或手动设置原点坐标

### Q: 编译失败
A: 确保先编译 `yunle_msgs` 包

```bash
colcon build --packages-select yunle_msgs
colcon build --packages-select vehicle_autoware_bridge
```

## 5. 参数调整

编辑 `config/vehicle_params.yaml`:

```yaml
autoware_to_chassis_converter:
  ros__parameters:
    speed_limit: 3.0      # 降低速度限制
    max_steer_angle: 20.0 # 降低转向限制
```

重启节点生效。

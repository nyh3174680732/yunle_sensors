# Vehicle-Autoware Bridge

真实车辆（云乐底盘 + 传感器）与 Autoware 自动驾驶系统的桥接包。

## 功能概述

本包提供了真实车辆与 Autoware Universe 之间的完整接口转换，包括：

### 1. 控制命令转换
- **Autoware → 底盘**: 将 Autoware 的标准控制命令转换为云乐底盘 ECU 命令
- 输入: `/control/command/control_cmd` (autoware_control_msgs/Control)
- 输出: `/ecu` (yunle_msgs/Ecu)
- 功能:
  - 速度单位转换 (m/s → km/h)
  - 转向角单位转换 (rad → deg)
  - 档位自动控制 (前进/后退/空档)
  - 速度和转向限制保护
  - 控制命令超时保护

### 2. 车辆状态转换
- **底盘 → Autoware**: 将底盘状态转换为 Autoware 标准车辆状态
- 输入: `/vehicle_status` (yunle_msgs/VehicleStatus)
- 输出:
  - `/vehicle/status/control_mode` (ControlModeReport)
  - `/vehicle/status/gear_status` (GearReport)
  - `/vehicle/status/steering_status` (SteeringReport)
  - `/vehicle/status/velocity_status` (VelocityReport)
  - `/vehicle/status/hazard_lights_status` (HazardLightsReport)
  - `/vehicle/status/turn_indicators_status` (TurnIndicatorsReport)

### 3. 传感器接口
- **IMU**: 重映射 `/imu/data` → `/sensing/imu/tamagawa/imu_raw`
- **GNSS**: 转换 GPS 经纬度 → 本地 ENU 坐标系位姿
  - 输入: `/sensing/gnss/navsatfix` (NavSatFix)
  - 输出: `/sensing/gnss/pose_with_covariance` (PoseWithCovarianceStamped)
- **激光雷达**: 直接使用 Livox 点云转换器输出的 `/sensing/lidar/top/pointcloud_raw_ex`

## 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                         Autoware Universe                        │
│  ┌──────────────────┐         ┌──────────────────┐              │
│  │  Planning Stack  │ ──────> │  Control Stack   │              │
│  └──────────────────┘         └──────────────────┘              │
│           │                            │                         │
│           │ /sensing/*                 │ /control/command/*      │
│           v                            v                         │
└───────────────────────────────────────────────────────────────┬─┘
                                                                  │
            ┌─────────────────────────────────────────────────────┘
            │  Vehicle-Autoware Bridge (本包)
            │  ┌──────────────────────────────────────────────┐
            │  │  autoware_to_chassis_converter               │
            │  │  chassis_to_autoware_converter               │
            │  │  imu_remapper                                 │
            │  │  gnss_to_autoware_converter                  │
            │  └──────────────────────────────────────────────┘
            │
            v
┌───────────────────────────────────────────────────────────────┐
│                    Real Vehicle Hardware                       │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐      │
│  │ Chassis  │  │  Livox   │  │   IMU    │  │   GNSS   │      │
│  │  Driver  │  │  LiDAR   │  │  AHRS    │  │ Receiver │      │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘      │
└───────────────────────────────────────────────────────────────┘
```

## 安装与编译

### 1. 依赖项
```bash
# ROS2 依赖
sudo apt install ros-humble-autoware-msgs ros-humble-serial

# 确保已经安装了传感器驱动包
# - sensor_bringup
# - livox_ros_driver2
# - fdilink_ahrs
# - gnss_converter
# - chassis_driver
```

### 2. 编译
```bash
cd ~/yunle_sensors  # 或你的工作空间路径
colcon build --packages-select vehicle_autoware_bridge
source install/setup.bash
```

## 使用方法

### 1. 启动桥接节点
```bash
# 启动所有桥接节点
ros2 launch vehicle_autoware_bridge vehicle_autoware_bridge.launch.py
```

### 2. 配置车辆参数
编辑配置文件: `config/vehicle_params.yaml`

关键参数:
- `wheel_base`: 车辆轴距 (0.501m)
- `speed_limit`: 速度限制 (5.0 m/s)
- `max_steer_angle`: 最大转向角 (30.0°)
- `origin_latitude/longitude`: GPS 原点坐标（首次运行后建议记录固定值）

### 3. 完整系统启动

#### 方式一: 分步启动（推荐用于调试）
```bash
# 终端 1: 启动传感器
ros2 launch sensor_bringup all_sensors_launch.py

# 终端 2: 启动底盘驱动
ros2 launch chassis_driver chassis_driver.launch.py

# 终端 3: 启动桥接节点
ros2 launch vehicle_autoware_bridge vehicle_autoware_bridge.launch.py

# 终端 4: 启动 Autoware
ros2 launch autoware_launch autoware.launch.xml ...
```

#### 方式二: 一键启动
创建整合 launch 文件，或使用脚本依次启动所有组件。

### 4. 话题检查
```bash
# 检查 Autoware 控制命令
ros2 topic echo /control/command/control_cmd

# 检查底盘状态
ros2 topic echo /vehicle_status

# 检查传感器数据
ros2 topic echo /sensing/lidar/top/pointcloud_raw_ex
ros2 topic echo /sensing/imu/tamagawa/imu_raw
ros2 topic echo /sensing/gnss/pose_with_covariance

# 检查车辆状态
ros2 topic echo /vehicle/status/velocity_status
```

## 话题映射关系

| Autoware 话题 | 方向 | 真实车辆话题 | 说明 |
|--------------|------|-------------|------|
| `/control/command/control_cmd` | → | `/ecu` | 控制命令 |
| `/vehicle/status/*` | ← | `/vehicle_status` | 车辆状态 |
| `/sensing/imu/tamagawa/imu_raw` | ← | `/imu/data` | IMU 数据 |
| `/sensing/gnss/pose_with_covariance` | ← | GPS 经纬度 | GNSS 定位 |
| `/sensing/lidar/top/pointcloud_raw_ex` | ← | Livox 直接输出 | 激光雷达 |

## 重要注意事项

### 安全相关
1. **速度限制**: 默认限制 5 m/s (~18 km/h)，根据实际场景调整
2. **控制超时**: 500ms 无控制命令自动发送停车指令
3. **急停功能**: 需要在上位机或底盘驱动中实现独立急停逻辑
4. **首次测试**: 建议在安全环境下以低速测试

### GPS 坐标系
1. 首次运行时会自动记录第一个 GPS 定位点作为原点
2. 建议记录原点坐标并写入配置文件，保证多次运行一致性
3. 如果更换测试地点，需要重新设置原点

### 坐标系变换
- 车辆本体坐标系: `base_link`
- IMU 坐标系: `tamagawa/imu_link`
- 激光雷达坐标系: `velodyne_top_base_link`
- GPS 坐标系: `gnss_link`
- 全局坐标系: `map`

需要配置正确的 TF 变换关系（通常在 sensor_kit_calibration.yaml 中定义）

## 故障排查

### 1. 车辆不响应控制命令
- 检查 `/ecu` 话题是否有数据: `ros2 topic hz /ecu`
- 检查底盘驱动是否正常运行
- 查看转换节点日志: `ros2 node list` 和 `ros2 node info`

### 2. GPS 定位不准确
- 检查 GPS 信号质量和卫星数量
- 确认原点坐标设置正确
- 检查坐标转换逻辑（小范围使用平面近似，大范围需要更精确的投影）

### 3. 速度或转向异常
- 检查单位转换是否正确（km/h ↔ m/s, deg ↔ rad）
- 检查车辆参数配置（轴距、轮距等）
- 查看限制保护是否触发

## 开发与调试

### 查看节点信息
```bash
# 列出所有桥接节点
ros2 node list | grep converter

# 查看节点参数
ros2 param list /autoware_to_chassis_converter

# 动态调整参数
ros2 param set /autoware_to_chassis_converter speed_limit 3.0
```

### 记录数据
```bash
# 记录所有相关话题
ros2 bag record /control/command/control_cmd /ecu /vehicle_status \
  /sensing/lidar/top/pointcloud_raw_ex /sensing/imu/tamagawa/imu_raw \
  /sensing/gnss/pose_with_covariance /vehicle/status/velocity_status
```

## 许可证
MIT License

## 联系方式
维护者: cplus (3174680732@qq.com)

# yunle_sensors - ROS 2 传感器包

## 项目简介
本项目包含多个 ROS 2 传感器驱动包，包括 AHRS、Livox 激光雷达等传感器的驱动程序。

## 依赖项
- ROS 2 (推荐 Humble 或更新版本)
- CMake
- colcon

## 编译说明

### 1. 安装 serial 包
`fdilink_ahrs` 包依赖于 `serial` 包，需要先编译安装：

```bash
# 编译 serial 包
colcon build --packages-select serial --symlink-install

# 加载环境变量
source install/setup.bash
```

### 2. 安装 Livox-SDK2
`livox_ros_driver2` 包依赖于 `Livox-SDK2`，需要先编译安装：

```bash
# 进入 Livox-SDK2 目录
cd src/Livox-SDK2/

# 创建构建目录
mkdir build
cd build

# 编译并安装
cmake .. && make -j
sudo make install

# 返回项目根目录
cd ../../..
```

### 3. 编译所有包
完成上述依赖安装后，编译整个工作空间：

```bash
# 编译所有包
colcon build --symlink-install

# 加载环境变量
source install/setup.bash
```

## 完整编译流程

```bash
# 1. 克隆项目
git clone https://github.com/nyh3174680732/yunle_sensors.git
cd yunle_sensors

# 2. 安装 serial 依赖
colcon build --packages-select serial --symlink-install
source install/setup.bash

# 3. 安装 Livox-SDK2 依赖
cd src/Livox-SDK2/
mkdir build && cd build
cmake .. && make -j
sudo make install
cd ../../..

# 4. 编译所有包
colcon build --symlink-install
source install/setup.bash
```

## 使用说明

编译完成后，可以使用以下命令启动相应的传感器节点：

```bash
# 启动 AHRS 节点
ros2 run fdilink_ahrs ahrs_driver

# 启动 Livox 激光雷达节点
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

## 故障排除

### 编译错误
- 确保已安装所有 ROS 2 依赖
- 检查 CMake 版本是否兼容
- 确保按照正确顺序编译依赖包

### 权限问题
如果 `sudo make install` 失败，请检查管理员权限。

### 环境变量
每次打开新终端时，记得执行：
```bash
source install/setup.bash
```

## 贡献
欢迎提交 Issue 和 Pull Request！

## 许可证
请查看各个子包的许可证文件。

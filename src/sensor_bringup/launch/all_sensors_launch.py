#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
All Sensors Launch File
同时启动所有传感器：GPS、IMU、Livox 激光雷达、单线雷达和点云转换器
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    生成 launch 描述，同时启动所有传感器节点：
    1. Livox MID360 激光雷达驱动
    2. 单线雷达驱动 (前后两个 Lakibeam1)
    3. 单线雷达 LaserScan 转 PointCloud2 转换器
    4. 点云转换为 Autoware 扩展格式 (top/left/right)
    5. 静态坐标变换 (velodyne_left/right_base_link -> laser_front/back_link)
    6. 点云格式转换器 (旧版, 已注释)
    7. IMU/AHRS 驱动
    8. GPS 转换节点
    """

    # ============== 声明启动参数 ==============
    # GPS 串口参数
    gps_serial_port_arg = DeclareLaunchArgument(
        'gps_serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for GNSS receiver'
    )

    gps_baudrate_arg = DeclareLaunchArgument(
        'gps_baudrate',
        default_value='115200',
        description='Baudrate for GNSS serial communication'
    )

    # IMU 串口参数
    imu_serial_port_arg = DeclareLaunchArgument(
        'imu_serial_port',
        default_value='/dev/ttyUSB1',
        description='Serial port for IMU/AHRS'
    )
    imu_baudrate_arg = DeclareLaunchArgument(
        'imu_baudrate',
        default_value='921600',
        description='Baudrate for IMU serial communication'
    )

    # 单线雷达参数
    lidar_front_frame_id_arg = DeclareLaunchArgument(
        'lidar_front_frame_id',
        default_value='laser_front_link',
        description='Frame ID for single-line lidars'
    )
    lidar_back_frame_id_arg = DeclareLaunchArgument(
        'lidar_back_frame_id',
        default_value='laser_back_link',
        description='Frame ID for single-line lidars'
    )
    lidar_front_topic_arg = DeclareLaunchArgument(
        'lidar_front_topic',
        default_value='laser/scan_front',
        description='Output topic for front single-line lidar'
    )
    lidar_back_topic_arg = DeclareLaunchArgument(
        'lidar_back_topic',
        default_value='laser/scan_back',
        description='Output topic for back single-line lidar'
    )
    lidar_hostip_arg = DeclareLaunchArgument(
        'lidar_hostip',
        default_value='192.168.1.100',
        description='Host IP for single-line lidars'
    )
    lidar_port_front_arg = DeclareLaunchArgument(
        'lidar_port_front',
        default_value='"2368"',
        description='Port for front single-line lidar'
    )
    lidar_port_back_arg = DeclareLaunchArgument(
        'lidar_port_back',
        default_value='"2369"',
        description='Port for back single-line lidar'
    )
    lidar_sensorip_front_arg = DeclareLaunchArgument(
        'lidar_sensorip_front',
        default_value='192.168.1.12',
        description='Sensor IP for front single-line lidar'
    )
    lidar_sensorip_back_arg = DeclareLaunchArgument(
        'lidar_sensorip_back',
        default_value='192.168.1.13',
        description='Sensor IP for back single-line lidar'
    )

    # ============== 1. Livox 激光雷达配置 ==============
    livox_config_path = get_package_share_directory('livox_ros_driver2')
    user_config_path = os.path.join(livox_config_path, 'config', 'MID360_config.json')
    livox_ros2_params = [
        {"xfer_format": 0},              # 0-Pointcloud2, 1-自定义点云格式
        {"multi_topic": 0},              # 0-所有雷达共享一个话题
        {"data_src": 0},                 # 0-激光雷达
        {"publish_freq": 10.0},          # 发布频率 10Hz
        {"output_data_type": 0},         # 输出数据类型
        {"frame_id": 'livox_frame'},     # 坐标系ID
        {"lvx_file_path": '/home/livox/livox_test.lvx'},
        {"user_config_path": user_config_path},
        {"cmdline_input_bd_code": 'livox0000000001'}
    ]

    livox_driver_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
    )

    # ============== 2. 单线雷达启动 ==============
    # 前单线雷达
    single_lidar_front_node = Node(
        package='lakibeam1',
        executable='lakibeam1_scan_node',
        name='richbeam_lidar_node_front',
        output='screen',
        parameters=[{
            'frame_id': LaunchConfiguration('lidar_front_frame_id'),
            'output_topic': LaunchConfiguration('lidar_front_topic'),
            'inverted': True,
            'hostip': LaunchConfiguration('lidar_hostip'),
            'port': LaunchConfiguration('lidar_port_front'),
            'sensorip': LaunchConfiguration('lidar_sensorip_front'),
            'angle_offset': 0,
            'scanfreq': '10',
            'filter': '3',
            'laser_enable': 'true',
            'scan_range_start': '45',
            'scan_range_stop': '315'
        }]
    )

    # 后单线雷达
    single_lidar_back_node = Node(
        package='lakibeam1',
        executable='lakibeam1_scan_node',
        name='richbeam_lidar_node_back',
        output='screen',
        parameters=[{
            'frame_id': LaunchConfiguration('lidar_back_frame_id'),
            'output_topic': LaunchConfiguration('lidar_back_topic'),
            'inverted': True,
            'hostip': LaunchConfiguration('lidar_hostip'),
            'port': LaunchConfiguration('lidar_port_back'),
            'sensorip': LaunchConfiguration('lidar_sensorip_back'),
            'angle_offset': 0,
            'scanfreq': '10',
            'filter': '3',
            'laser_enable': 'true',
            'scan_range_start': '45',
            'scan_range_stop': '315'
        }]
    )

    # ============== 3. 单线雷达 LaserScan 转 PointCloud2 ==============
    # 前单线雷达转换
    laserscan_to_pointcloud_front_node = Node(
        package='pointcloud_to_laserscan',
        executable='laserscan_to_pointcloud_node',
        name='laserscan_to_pointcloud_front',
        output='screen',
        remappings=[
            ('scan_in', LaunchConfiguration('lidar_front_topic')),
            ('cloud', 'laser/cloud_front')
        ],
        parameters=[{
            'target_frame': LaunchConfiguration('lidar_front_frame_id'),
            'transform_tolerance': 0.01
        }]
    )

    # 后单线雷达转换
    laserscan_to_pointcloud_back_node = Node(
        package='pointcloud_to_laserscan',
        executable='laserscan_to_pointcloud_node',
        name='laserscan_to_pointcloud_back',
        output='screen',
        remappings=[
            ('scan_in', LaunchConfiguration('lidar_back_topic')),
            ('cloud', 'laser/cloud_back')
        ],
        parameters=[{
            'target_frame': LaunchConfiguration('lidar_back_frame_id'),
            'transform_tolerance': 0.01
        }]
    )

    # ============== 4. 点云转换为 Autoware 扩展格式 ==============
    pointcloud_to_autoware_converter_node = Node(
        package='vehicle_autoware_bridge',
        executable='pointcloud_to_autoware_converter_node',
        name='pointcloud_to_autoware_converter',
        output='screen',
        parameters=[{
            'livox_input_topic': '/livox/lidar',
            'top_output_topic': '/sensing/lidar/top/pointcloud_raw_ex',
            'top_frame_id': 'velodyne_top_base_link',
            'front_input_topic': 'laser/cloud_front',
            'left_output_topic': '/sensing/lidar/left/pointcloud_raw_ex',
            'left_frame_id': 'velodyne_left_base_link',
            'back_input_topic': 'laser/cloud_back',
            'right_output_topic': '/sensing/lidar/right/pointcloud_raw_ex',
            'right_frame_id': 'velodyne_right_base_link',
            'use_system_time': True
        }]
    )

    # ============== 5. 静态坐标变换 ==============
    # velodyne_left_base_link -> laser_front_link (原地变换)
    static_tf_left_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_left_to_front',
        arguments=['0', '0', '0', '0', '0', '0', 'velodyne_left_base_link', 'laser_front_link']
    )

    # velodyne_right_base_link -> laser_back_link (原地变换)
    static_tf_right_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_right_to_back',
        arguments=['0', '0', '0', '0', '0', '0', 'velodyne_right_base_link', 'laser_back_link']
    )

    # # ============== 6. 点云转换器配置 (旧版, 已注释) ==============
    # pointcloud_converter_node = Node(
    #     package='livox_ros_driver2',
    #     executable='pointcloud_converter_node',
    #     name='livox_pointcloud_converter',
    #     output='screen',
    #     parameters=[{
    #         'input_topic': '/livox/lidar',
    #         'output_topic': '/sensing/lidar/top/pointcloud_raw_ex',
    #         'output_frame_id': 'velodyne_top_base_link'
    #     }]
    # )

    # ============== 7. IMU/AHRS 驱动配置 ==============
    imu_driver_node = Node(
        package='fdilink_ahrs',
        executable='ahrs_driver_node',
        name='ahrs_driver_node',
        output='screen',
        parameters=[{
            'if_debug_': False,
            'serial_port_': LaunchConfiguration('imu_serial_port'),
            'serial_baud_': LaunchConfiguration('imu_baudrate'),
            'imu_topic': '/sensing/imu/tamagawa/imu_raw',
            'imu_frame_id_': 'tamagawa/imu_link',
            'mag_pose_2d_topic': '/sensing/imu/mag_pose_2d',
            'Magnetic_topic': '/sensing/imu/magnetic',
            'Euler_angles_topic': '/sensing/imu/euler_angles',
            'gps_topic': '/sensing/imu/gps_fix',
            'twist_topic': '/sensing/imu/twist',
            'NED_odom_topic': '/sensing/imu/ned_odometry',
            'device_type_': 1
        }]
    )

    # ============== 8. GPS 转换器配置 ==============
    gnss_driver_node = Node(
        package='gnss',
        executable='gnss_driver_node',
        name='gnss_driver_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('gps_serial_port'),
            'baudrate': LaunchConfiguration('gps_baudrate'),
            'frame_id': 'gnss_link',
        }]
    )

    # ============== 构建 Launch Description ==============
    return LaunchDescription([
        # 声明参数
        gps_serial_port_arg,
        gps_baudrate_arg,
        imu_serial_port_arg,
        imu_baudrate_arg,
        lidar_front_frame_id_arg,
        lidar_back_frame_id_arg,
        lidar_front_topic_arg,
        lidar_back_topic_arg,
        lidar_hostip_arg,
        lidar_port_front_arg,
        lidar_port_back_arg,
        lidar_sensorip_front_arg,
        lidar_sensorip_back_arg,

        # 启动节点
        livox_driver_node,
        single_lidar_front_node,
        single_lidar_back_node,
        laserscan_to_pointcloud_front_node,
        laserscan_to_pointcloud_back_node,
        pointcloud_to_autoware_converter_node,
        static_tf_left_node,
        static_tf_right_node,
        imu_driver_node,
        gnss_driver_node,
    ])

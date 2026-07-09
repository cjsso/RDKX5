from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 启动参数声明
    serial_port_param = DeclareLaunchArgument(
        'serial_port',
        default_value='ttyS1',
        description='usb bus name, e.g. ttyS1'
    )

    vel_x_correct_param = DeclareLaunchArgument(
        'vel_x_correct',
        default_value='0.898',
        description='correct factor vx, e.g. 0.9'
    )

    vel_theta_correct_param = DeclareLaunchArgument(
        'vel_theta_correct',
        default_value='0.874',
        description='correct factor vth, e.g. 0.9'
    )

    auto_stop_enable_param = DeclareLaunchArgument(
        'auto_stop_enable',
        default_value='true',
        description='auto stop if no cmd received, true or true'
    )

    imu_use_switch_param = DeclareLaunchArgument(
        'imu_use_switch',
        default_value='false',
        description='if has imu sensor to drive'
    )

    odom_tf_pub_param = DeclareLaunchArgument(
        'odom_tf_pub',
        default_value='true',
        description='publishing odom to base_footprint tf, true or true'
    )

    # 配置变量
    serial_port_cfg = LaunchConfiguration('serial_port')
    vel_x_cfg = LaunchConfiguration('vel_x_correct')
    vel_theta_cfg = LaunchConfiguration('vel_theta_correct')
    auto_stop_cfg = LaunchConfiguration('auto_stop_enable')
    imu_switch_cfg = LaunchConfiguration('imu_use_switch')
    odom_pub_cfg = LaunchConfiguration('odom_tf_pub')

    # 底盘节点
    chassis_driver_node = Node(
        package='base',
        executable='originbot_base',
        output='screen',
        parameters=[{
            'port_name': serial_port_cfg,
            'correct_factor_vx': vel_x_cfg,
            'correct_factor_vth': vel_theta_cfg,
            'auto_stop_on': auto_stop_cfg,
            'use_imu': imu_switch_cfg,
            'pub_odom': odom_pub_cfg,
        }]
    )

    # base_footprint 静态坐标变换
    footprint_link_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["--x", "0.0", "--y", "0.0", "--z", "0.05325", "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0", "--frame-id", "base_footprint", "--child-frame-id", "base_link"]
    )

    # imu 静态坐标变换
    imu_link_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["--x", "0.0", "--y", "0.0", "--z", "0.0", "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0", "--frame-id", "base_link", "--child-frame-id", "imu_link"]
    )

    # 启动列表
    launch_item_list = [
        serial_port_param,
        vel_x_correct_param,
        vel_theta_correct_param,
        auto_stop_enable_param,
        imu_use_switch_param,
        odom_tf_pub_param,
        chassis_driver_node,
        footprint_link_tf_node,
        imu_link_tf_node
    ]

    return LaunchDescription(launch_item_list)
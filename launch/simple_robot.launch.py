#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 声明启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    control_rate_arg = DeclareLaunchArgument(
        'control_rate',
        default_value='100.0',
        description='Control loop rate in Hz'
    )
    
    sim_rate_arg = DeclareLaunchArgument(
        'sim_rate',
        default_value='1000.0',
        description='Simulation rate in Hz'
    )
    
    # 获取参数值
    use_sim_time = LaunchConfiguration('use_sim_time')
    control_rate = LaunchConfiguration('control_rate')
    sim_rate = LaunchConfiguration('sim_rate')
    
    # 简化机器人控制器节点
    simple_controller_node = Node(
        package='ros2_mujoco_robot',
        executable='simple_robot_controller',
        name='simple_robot_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'control_rate': control_rate,
            'sim_rate': sim_rate,
            'joint_names': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        }]
    )
    
    # 关节状态发布器节点
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'source_list': ['/joint_states']
        }]
    )
    
    # 创建启动描述
    ld = LaunchDescription()
    
    # 添加参数
    ld.add_action(use_sim_time_arg)
    ld.add_action(control_rate_arg)
    ld.add_action(sim_rate_arg)
    
    # 添加节点
    ld.add_action(simple_controller_node)
    ld.add_action(joint_state_publisher_node)
    
    return ld 
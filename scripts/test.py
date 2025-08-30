#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import time
import math

from std_msgs.msg import Float64MultiArray
 
class TestController(Node):
    """
    测试控制器
    测试关节角度、速度、扭矩控制
    """
    
    def __init__(self):
        super().__init__('test_controller')
        
        # 关节名称
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # 创建发布者
        self.position_pub = self.create_publisher(
            Float64MultiArray, '/joint_position_commands', 10)
        
        self.velocity_pub = self.create_publisher(
            Float64MultiArray, '/joint_velocity_commands', 10)
        
        self.torque_pub = self.create_publisher(
            Float64MultiArray, '/joint_torque_commands', 10)
        
        # 测试状态
        self.test_counter = 0
        self.test_timer = self.create_timer(5.0, self.run_test)
        
        self.get_logger().info('测试控制器已启动')
    
    def run_test(self):
        """运行测试"""
        self.test_counter += 1
        
        if self.test_counter == 1:
            self.test_position_control()
        elif self.test_counter == 2:
            self.test_velocity_control()
        elif self.test_counter == 3:
            self.test_torque_control()
        else:
            self.get_logger().info('所有测试完成')
            self.test_timer.cancel()
    
    def test_position_control(self):
        """测试关节角度控制"""
        self.get_logger().info('测试1: 关节角度控制')
        
        # 设置目标角度
        target_positions = [0.5, -0.3, 0.8, -0.2, 0.4, -0.6]
        
        cmd_msg = Float64MultiArray()
        cmd_msg.data = target_positions
        self.position_pub.publish(cmd_msg)
        
        self.get_logger().info(f'设置目标角度: {target_positions}')
    
    def test_velocity_control(self):
        """测试关节速度控制"""
        self.get_logger().info('测试2: 关节速度控制')
        
        # 设置目标速度
        target_velocities = [0.1, -0.1, 0.2, -0.1, 0.15, -0.15]
        
        cmd_msg = Float64MultiArray()
        cmd_msg.data = target_velocities
        self.velocity_pub.publish(cmd_msg)
        
        self.get_logger().info(f'设置目标速度: {target_velocities}')
    
    def test_torque_control(self):
        """测试关节扭矩控制"""
        self.get_logger().info('测试3: 关节扭矩控制')
        
        # 设置目标扭矩
        target_torques = [1.0, -1.0, 2.0, -1.0, 1.5, -1.5]
        
        cmd_msg = Float64MultiArray()
        cmd_msg.data = target_torques
        self.torque_pub.publish(cmd_msg)
        
        self.get_logger().info(f'设置目标扭矩: {target_torques}')

def main(args=None):
    rclpy.init(args=args)
    
    tester = TestController()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
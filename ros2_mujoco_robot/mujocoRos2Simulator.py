#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import threading
import time
import os

import mujoco
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped

class MujocoRos2Simulator(Node):
    """
    提供关节角度、速度、扭矩控制接口
    """
    
    def __init__(self):
        super().__init__('robot_controller')
        
        # 初始化参数
        self.declare_parameter('joint_names', ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'])
        self.declare_parameter('control_rate', 100.0)  # Hz
        self.declare_parameter('sim_rate', 1000.0)     # Hz
        
        self.joint_names = self.get_parameter('joint_names').value
        self.control_rate = self.get_parameter('control_rate').value
        self.sim_rate = self.get_parameter('sim_rate').value
        
        # 初始化MuJoCo
        self.setup_mujoco()
        
        # 控制状态
        self.control_mode = 'position'  # 'position', 'velocity', 'torque'
        self.target_positions = np.zeros(len(self.joint_names))
        self.target_velocities = np.zeros(len(self.joint_names))
        self.target_torques = np.zeros(len(self.joint_names))
        
        # 当前状态
        self.current_positions = np.zeros(len(self.joint_names))
        self.current_velocities = np.zeros(len(self.joint_names))
        
        # 设置ROS接口
        self.setup_ros_interface()
        
        # 启动仿真线程
        self.is_running = True
        self.sim_thread = threading.Thread(target=self.simulation_loop)
        self.sim_thread.daemon = True
        self.sim_thread.start()
        
        # 启动控制定时器
        self.control_timer = self.create_timer(1.0/self.control_rate, self.control_loop)
        
        self.get_logger().info('机器人控制器已启动')
    
    def setup_mujoco(self):
        """设置MuJoCo仿真"""
        try:
            # 加载用户自定义模型
            package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            models_dir = os.path.join(package_path, 'models')
            
            # 查找模型文件
            model_files = [f for f in os.listdir(models_dir) if f.endswith('.xml')]
            
            if not model_files:
                self.get_logger().error('在models目录中未找到.xml模型文件')
                raise FileNotFoundError('未找到模型文件')
            
            # 使用第一个找到的模型文件
            model_file = os.path.join(models_dir, model_files[0])
            self.get_logger().info(f'加载模型文件: {model_file}')
            
            self.model = mujoco.MjModel.from_xml_path(model_file)
            self.data = mujoco.MjData(self.model)
            
            # 验证关节名称
            available_joints = []
            for i in range(self.model.njnt):
                joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
                if joint_name:
                    available_joints.append(joint_name)
            
            self.get_logger().info(f'模型中的关节: {available_joints}')
            
            # 检查指定的关节名称是否存在于模型中
            for joint_name in self.joint_names:
                if joint_name not in available_joints:
                    self.get_logger().warn(f'关节 {joint_name} 在模型中未找到')
            
            self.get_logger().info(f'成功加载MuJoCo模型: {len(self.joint_names)} 个关节')
            
        except Exception as e:
            self.get_logger().error(f'MuJoCo初始化失败: {e}')
            raise
    
    def setup_ros_interface(self):
        """设置ROS接口"""
        # 发布者
        self.joint_state_pub = self.create_publisher(
            JointState, '/joint_states', 10)
        
        self.robot_pose_pub = self.create_publisher(
            PoseStamped, '/robot_pose', 10)
        
        # 订阅者 - 关节角度控制
        self.joint_position_sub = self.create_subscription(
            Float64MultiArray, '/joint_position_commands', 
            self.joint_position_callback, 10)
        
        # 订阅者 - 关节速度控制
        self.joint_velocity_sub = self.create_subscription(
            Float64MultiArray, '/joint_velocity_commands', 
            self.joint_velocity_callback, 10)
        
        # 订阅者 - 关节扭矩控制
        self.joint_torque_sub = self.create_subscription(
            Float64MultiArray, '/joint_torque_commands', 
            self.joint_torque_callback, 10)
    
    def joint_position_callback(self, msg):
        """关节角度控制回调"""
        if len(msg.data) == len(self.joint_names):
            self.control_mode = 'position'
            self.target_positions = np.array(msg.data)
            self.get_logger().info(f'设置关节角度: {self.target_positions}')
        else:
            self.get_logger().warn(f'关节角度命令维度不匹配: 期望{len(self.joint_names)}, 实际{len(msg.data)}')
    
    def joint_velocity_callback(self, msg):
        """关节速度控制回调"""
        if len(msg.data) == len(self.joint_names):
            self.control_mode = 'velocity'
            self.target_velocities = np.array(msg.data)
            self.get_logger().info(f'设置关节速度: {self.target_velocities}')
        else:
            self.get_logger().warn(f'关节速度命令维度不匹配: 期望{len(self.joint_names)}, 实际{len(msg.data)}')
    
    def joint_torque_callback(self, msg):
        """关节扭矩控制回调"""
        if len(msg.data) == len(self.joint_names):
            self.control_mode = 'torque'
            self.target_torques = np.array(msg.data)
            self.get_logger().info(f'设置关节扭矩: {self.target_torques}')
        else:
            self.get_logger().warn(f'关节扭矩命令维度不匹配: 期望{len(self.joint_names)}, 实际{len(msg.data)}')
    
    def simulation_loop(self):
        """仿真主循环"""
        dt = 1.0 / self.sim_rate
        
        while self.is_running and rclpy.ok():
            # 运行仿真步
            mujoco.mj_step(self.model, self.data)
            
            # 控制仿真频率
            time.sleep(dt)
    
    def control_loop(self):
        """控制循环"""
        # 更新当前状态
        self.current_positions = self.data.qpos[:len(self.joint_names)].copy()
        self.current_velocities = self.data.qvel[:len(self.joint_names)].copy()
        
        # 根据控制模式设置控制命令
        if self.control_mode == 'position':
            # 简单的PD控制器
            kp = 100.0
            kd = 10.0
            position_error = self.target_positions - self.current_positions
            velocity_error = -self.current_velocities
            control_output = kp * position_error + kd * velocity_error
            self.data.ctrl[:len(self.joint_names)] = control_output
            
        elif self.control_mode == 'velocity':
            # 简单的P控制器
            kp = 50.0
            velocity_error = self.target_velocities - self.current_velocities
            control_output = kp * velocity_error
            self.data.ctrl[:len(self.joint_names)] = control_output
            
        elif self.control_mode == 'torque':
            # 直接扭矩控制
            self.data.ctrl[:len(self.joint_names)] = self.target_torques
        
        # 发布关节状态
        self.publish_joint_state()
        
        # 发布机器人位姿
        self.publish_robot_pose()
    
    def publish_joint_state(self):
        """发布关节状态"""
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = self.joint_names
        joint_msg.position = self.current_positions.tolist()
        joint_msg.velocity = self.current_velocities.tolist()
        joint_msg.effort = self.data.qfrc_applied[:len(self.joint_names)].tolist()
        
        self.joint_state_pub.publish(joint_msg)
    
    def publish_robot_pose(self):
        """发布机器人位姿"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "world"
        
        # 尝试获取末端执行器位姿
        try:
            end_effector_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "end_effector")
            if end_effector_id >= 0:
                pos = self.data.xpos[end_effector_id]
                quat = self.data.xquat[end_effector_id]
                
                pose_msg.pose.position.x = pos[0]
                pose_msg.pose.position.y = pos[1]
                pose_msg.pose.position.z = pos[2]
                pose_msg.pose.orientation.w = quat[0]
                pose_msg.pose.orientation.x = quat[1]
                pose_msg.pose.orientation.y = quat[2]
                pose_msg.pose.orientation.z = quat[3]
                
                self.robot_pose_pub.publish(pose_msg)
        except:
            # 如果找不到末端执行器，使用第一个刚体的位姿
            if self.model.nbody > 1:  # 跳过world刚体
                pos = self.data.xpos[1]
                quat = self.data.xquat[1]
                
                pose_msg.pose.position.x = pos[0]
                pose_msg.pose.position.y = pos[1]
                pose_msg.pose.position.z = pos[2]
                pose_msg.pose.orientation.w = quat[0]
                pose_msg.pose.orientation.x = quat[1]
                pose_msg.pose.orientation.y = quat[2]
                pose_msg.pose.orientation.z = quat[3]
                
                self.robot_pose_pub.publish(pose_msg)
    
    def shutdown(self):
        """关闭控制器"""
        self.is_running = False
        if hasattr(self, 'sim_thread'):
            self.sim_thread.join(timeout=1.0)

def main(args=None):
    rclpy.init(args=args)
    
    controller = MujocoRos2Simulator()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.shutdown()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
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

class MujocoSimulator(Node):
    """
    提供关节角度、速度、扭矩控制接口
    """
    
    def __init__(self):
        super().__init__('simple_robot_controller')
        
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
        
        self.get_logger().info('简化机器人控制器已启动')
    
    def setup_mujoco(self):
        """设置MuJoCo仿真"""
        try:
            # 创建简单的机器人模型
            self.create_simple_robot_model()
            
            # 加载模型
            package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            model_file = os.path.join(package_path, 'models', 'simple_robot.xml')
            
            self.model = mujoco.MjModel.from_xml_path(model_file)
            self.data = mujoco.MjData(self.model)
            
            self.get_logger().info(f'成功加载MuJoCo模型: {len(self.joint_names)} 个关节')
            
        except Exception as e:
            self.get_logger().error(f'MuJoCo初始化失败: {e}')
            raise
    
    def create_simple_robot_model(self):
        """创建简单的机器人模型"""
        package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        models_dir = os.path.join(package_path, 'models')
        os.makedirs(models_dir, exist_ok=True)
        
        # 6自由度机械臂模型
        robot_xml = '''<?xml version="1.0" encoding="UTF-8"?>
<mujoco model="simple_robot">
  <compiler angle="radian" coordinate="local"/>
  <option timestep="0.001" iterations="50" solver="Newton" tolerance="1e-10"/>
  
  <default>
    <default class="link">
      <geom type="cylinder" size="0.05 0.1" rgba="0.7 0.7 0.7 1"/>
      <joint type="hinge" axis="0 0 1" damping="0.1"/>
    </default>
  </default>
  
  <worldbody>
    <light directional="true" diffuse=".8 .8 .8" specular=".2 .2 .2" castshadow="false" pos="0 0 5" dir="0 0 -1"/>
    <geom type="plane" size="10 10 0.1" pos="0 0 -0.1" rgba="0.8 0.8 0.8 1"/>
    
    <!-- 机器人基座 -->
    <body name="base" pos="0 0 0">
      <geom type="cylinder" size="0.1 0.05" rgba="0.3 0.3 0.3 1"/>
      
      <!-- 关节1 -->
      <body name="link1" pos="0 0 0.1">
        <geom class="link" size="0.05 0.15"/>
        <joint name="joint1" class="link"/>
        
        <!-- 关节2 -->
        <body name="link2" pos="0 0 0.15">
          <geom class="link" size="0.05 0.15"/>
          <joint name="joint2" class="link"/>
          
          <!-- 关节3 -->
          <body name="link3" pos="0 0 0.15">
            <geom class="link" size="0.05 0.15"/>
            <joint name="joint3" class="link"/>
            
            <!-- 关节4 -->
            <body name="link4" pos="0 0 0.15">
              <geom class="link" size="0.05 0.15"/>
              <joint name="joint4" class="link"/>
              
              <!-- 关节5 -->
              <body name="link5" pos="0 0 0.15">
                <geom class="link" size="0.05 0.15"/>
                <joint name="joint5" class="link"/>
                
                <!-- 关节6 -->
                <body name="link6" pos="0 0 0.15">
                  <geom class="link" size="0.05 0.15"/>
                  <joint name="joint6" class="link"/>
                  
                  <!-- 末端执行器 -->
                  <body name="end_effector" pos="0 0 0.1">
                    <geom type="sphere" size="0.03" rgba="0.2 0.8 0.2 1"/>
                  </body>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
  
  <actuator>
    <motor name="motor1" joint="joint1" gear="100"/>
    <motor name="motor2" joint="joint2" gear="100"/>
    <motor name="motor3" joint="joint3" gear="100"/>
    <motor name="motor4" joint="joint4" gear="100"/>
    <motor name="motor5" joint="joint5" gear="100"/>
    <motor name="motor6" joint="joint6" gear="100"/>
  </actuator>
  
  <sensor>
    <jointpos name="joint1_pos" joint="joint1"/>
    <jointpos name="joint2_pos" joint="joint2"/>
    <jointpos name="joint3_pos" joint="joint3"/>
    <jointpos name="joint4_pos" joint="joint4"/>
    <jointpos name="joint5_pos" joint="joint5"/>
    <jointpos name="joint6_pos" joint="joint6"/>
    
    <jointvel name="joint1_vel" joint="joint1"/>
    <jointvel name="joint2_vel" joint="joint2"/>
    <jointvel name="joint3_vel" joint="joint3"/>
    <jointvel name="joint4_vel" joint="joint4"/>
    <jointvel name="joint5_vel" joint="joint5"/>
    <jointvel name="joint6_vel" joint="joint6"/>
  </sensor>
</mujoco>'''
        
        model_file = os.path.join(models_dir, 'simple_robot.xml')
        with open(model_file, 'w') as f:
            f.write(robot_xml)
        
        self.get_logger().info(f'创建了简单机器人模型: {model_file}')
    
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
        
        # 获取末端执行器位姿
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
    
    def shutdown(self):
        """关闭控制器"""
        self.is_running = False
        if hasattr(self, 'sim_thread'):
            self.sim_thread.join(timeout=1.0)

def main(args=None):
    rclpy.init(args=args)
    
    controller = MujocoSimulator()
    
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
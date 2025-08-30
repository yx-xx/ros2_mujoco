# ROS2 MuJoCo 机器人仿真

这是一个基于ROS2和MuJoCo的机器人仿真，提供关节控制接口。

## 功能特性

- **关节角度控制**: 通过PD控制器实现精确的位置控制
- **关节速度控制**: 通过P控制器实现速度控制
- **关节扭矩控制**: 直接扭矩控制
- **MuJoCo仿真**: 高性能物理仿真
- **ROS2集成**: 简洁的ROS2接口

## 系统要求

- Ubuntu 22.04+
- ROS2 Humble+
- Python 3.8+
- MuJoCo 2.3.0+

## 快速开始

### 1. 安装依赖

```bash
# 安装ROS2控制包
sudo apt install ros-humble-control-msgs

# 安装MuJoCo
pip install mujoco numpy
```

# 激活conda环境
conda activate ros2_mujoco

# 设置ROS2环境
source /opt/ros/humble/setup.bash




### 2. 构建包

```bash
colcon build --packages-select ros2_mujoco_robot
source install/setup.bash
```

### 3. 启动仿真

```bash
ros2 launch ros2_mujoco_robot simple_robot.launch.py
```

### 4. 运行测试

```bash
# 在另一个终端
ros2 run ros2_mujoco_robot simple_test
```

## 控制接口

### 话题 (Topics)

#### 输入话题 (控制命令)
- `/joint_position_commands` (std_msgs/Float64MultiArray): 关节角度控制
- `/joint_velocity_commands` (std_msgs/Float64MultiArray): 关节速度控制  
- `/joint_torque_commands` (std_msgs/Float64MultiArray): 关节扭矩控制

#### 输出话题 (状态反馈)
- `/joint_states` (sensor_msgs/JointState): 关节状态
- `/robot_pose` (geometry_msgs/PoseStamped): 机器人位姿

## 使用示例

### Python控制示例

```python
import rclpy
from std_msgs.msg import Float64MultiArray

# 初始化ROS2
rclpy.init()
node = rclpy.create_node('test_controller')

# 创建发布者
position_pub = node.create_publisher(Float64MultiArray, '/joint_position_commands', 10)
velocity_pub = node.create_publisher(Float64MultiArray, '/joint_velocity_commands', 10)
torque_pub = node.create_publisher(Float64MultiArray, '/joint_torque_commands', 10)

# 关节角度控制
position_cmd = Float64MultiArray()
position_cmd.data = [0.5, -0.3, 0.8, -0.2, 0.4, -0.6]
position_pub.publish(position_cmd)

# 关节速度控制
velocity_cmd = Float64MultiArray()
velocity_cmd.data = [0.1, -0.1, 0.2, -0.1, 0.15, -0.15]
velocity_pub.publish(velocity_cmd)

# 关节扭矩控制
torque_cmd = Float64MultiArray()
torque_cmd.data = [1.0, -1.0, 2.0, -1.0, 1.5, -1.5]
torque_pub.publish(torque_cmd)
```

### 命令行控制示例

```bash
# 关节角度控制
ros2 topic pub /joint_position_commands std_msgs/msg/Float64MultiArray "data: [0.5, -0.3, 0.8, -0.2, 0.4, -0.6]"

# 关节速度控制
ros2 topic pub /joint_velocity_commands std_msgs/msg/Float64MultiArray "data: [0.1, -0.1, 0.2, -0.1, 0.15, -0.15]"

# 关节扭矩控制
ros2 topic pub /joint_torque_commands std_msgs/msg/Float64MultiArray "data: [1.0, -1.0, 2.0, -1.0, 1.5, -1.5]"
```

## 配置参数

### 启动参数

- `control_rate`: 控制循环频率 (默认: 100 Hz)
- `sim_rate`: 仿真频率 (默认: 1000 Hz)
- `joint_names`: 关节名称列表

### 控制器参数

- 位置控制: PD控制器 (kp=100, kd=10)
- 速度控制: P控制器 (kp=50)
- 扭矩控制: 直接控制

## 监控状态

```bash
# 查看关节状态
ros2 topic echo /joint_states

# 查看机器人位姿
ros2 topic echo /robot_pose

# 查看话题列表
ros2 topic list
```

## 故障排除

### 常见问题

1. **MuJoCo初始化失败**
   - 检查MuJoCo是否正确安装
   - 确保模型文件路径正确

2. **控制命令无响应**
   - 检查话题名称是否正确
   - 确认控制器正在运行
   - 验证消息格式

3. **仿真不稳定**
   - 降低仿真频率
   - 调整控制器参数
   - 检查系统资源

## 文件结构

```
ros2_mujoco_robot/
├── ros2_mujoco_robot/
│   ├── simple_robot_controller.py    # 主控制器
│   └── simple_test.py               # 测试脚本
├── launch/
│   └── simple_robot.launch.py       # 启动文件
├── models/
│   └── simple_robot.xml             # 机器人模型
└── scripts/
    └── simple_test.py               # 测试脚本
```

## 许可证

MIT License 
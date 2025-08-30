# 模型文件目录

请将您的MuJoCo模型文件（.xml格式）放置在此目录中。

## 支持的模型格式
- MuJoCo XML格式 (.xml)

## 模型要求
1. 模型文件必须是有效的MuJoCo XML格式
2. 建议包含以下关节名称（如果您的模型使用不同的关节名称，请相应修改launch文件中的参数）：
   - joint1
   - joint2
   - joint3
   - joint4
   - joint5
   - joint6

## 使用说明
1. 将您的模型文件重命名为 `robot.xml` 或任何您喜欢的名称
2. 控制器会自动加载models目录中的第一个.xml文件
3. 如果您的模型使用不同的关节名称，请修改 `launch/robot.launch.py` 中的 `joint_names` 参数

## 示例模型结构
```xml
<?xml version="1.0" encoding="UTF-8"?>
<mujoco model="robot">
  <compiler angle="radian" coordinate="local" meshdir="meshes/"/>
  
  <default>
    <default class="link">
      <geom type="cylinder" size="0.05 0.1" rgba="0.8 0.8 0.8 1"/>
    </default>
    <default class="joint">
      <joint type="hinge" axis="0 0 1" damping="0.1"/>
    </default>
  </default>
  
  <worldbody>
    <body name="base" pos="0 0 0">
      <geom class="link" type="box" size="0.1 0.1 0.05" rgba="0.5 0.5 0.5 1"/>
      
      <body name="link1" pos="0 0 0.1">
        <joint name="joint1" class="joint"/>
        <geom class="link"/>
        
        <body name="link2" pos="0 0 0.2">
          <joint name="joint2" class="joint"/>
          <geom class="link"/>
          
          <!-- 继续添加更多关节... -->
        </body>
      </body>
    </body>
  </worldbody>
  
  <actuator>
    <motor name="motor1" joint="joint1" gear="100"/>
    <motor name="motor2" joint="joint2" gear="100"/>
    <!-- 继续添加更多执行器... -->
  </actuator>
</mujoco>
```

## 注意事项
- 确保模型文件中的关节名称与控制器中配置的关节名称一致
- 如果模型包含mesh文件，请确保mesh文件路径正确
- 建议在模型文件中定义适当的执行器（actuator）以支持控制 
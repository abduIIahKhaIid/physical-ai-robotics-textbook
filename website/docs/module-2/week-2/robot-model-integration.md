---
title: "1.2 Robot Model Integration in Gazebo"
description: "Import and configure robot models in Gazebo using URDF and SDF formats."
tags: [robot-model, integration, urdf, sdf, gazebo, simulation, robotics]
learning-objectives:
  - "Import URDF and SDF robot models into Gazebo"
  - "Configure joint properties and transmissions for simulation"
  - "Set up robot state publishers for proper TF transforms"
  - "Spawn custom robots in simulation environment"
  - "Validate robot model behavior in simulation"
---

# 1.2 Robot Model Integration in Gazebo

## Learning Objectives

By the end of this chapter, you will be able to:
- Import URDF and SDF robot models into Gazebo
- Configure joint properties and transmissions for simulation
- Set up robot state publishers for proper TF transforms
- Spawn custom robots in simulation environment
- Validate robot model behavior in simulation

## Introduction

Robot model integration is a crucial step in simulation that enables you to test your robot designs and control algorithms in a virtual environment. This chapter covers the process of bringing robot models into Gazebo using both URDF (Unified Robot Description Format) and SDF (Simulation Description Format), with a focus on ensuring proper simulation behavior.

## 1.2.1 URDF vs. SDF Formats

### URDF (Unified Robot Description Format)
URDF is the standard format for describing robots in ROS. It's XML-based and focuses on robot structure and kinematic properties.

**Advantages of URDF:**
- Standard in ROS ecosystem
- Extensive tooling support
- Easy to understand and modify
- Integration with ROS toolchain

**Limitations of URDF:**
- Limited dynamic simulation features
- Cannot describe multiple robots in one file
- Less flexible for complex simulation scenarios

### SDF (Simulation Description Format)
SDF is Gazebo's native format, also XML-based but designed specifically for simulation.

**Advantages of SDF:**
- Full simulation feature support
- Can describe complete worlds with multiple robots
- More flexible for simulation-specific features
- Native Gazebo support

**Limitations of SDF:**
- More complex than URDF
- Less tooling outside of Gazebo
- Less standard outside of simulation

### Conversion Between Formats
Gazebo can automatically convert URDF to SDF using the `libgazebo_ros_xacro` plugin.

## 1.2.2 URDF Robot Model Structure

### Basic URDF Structure
```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Links define the rigid bodies of the robot -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Joints connect links together -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 0.3 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>
</robot>
```

### Key Components
- **Links**: Represent rigid bodies with visual, collision, and inertial properties
- **Joints**: Define connections between links with specific degrees of freedom
- **Materials**: Define visual appearance
- **Transmissions**: Define how joints connect to actuators (important for simulation)

## 1.2.3 SDF Robot Model Structure

### Basic SDF Structure
```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="my_robot">
    <!-- Model properties -->
    <static>false</static>
    <allow_auto_disable>true</allow_auto_disable>

    <!-- Links -->
    <link name="base_link">
      <pose>0 0 0.3 0 0 0</pose>
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>0.4</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.4</iyy>
          <iyz>0</iyz>
          <izz>0.2</izz>
        </inertia>
      </inertial>

      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.2</radius>
            <length>0.6</length>
          </cylinder>
        </geometry>
      </collision>

      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.2</radius>
            <length>0.6</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0 0 0.8 1</ambient>
          <diffuse>0 0 0.8 1</diffuse>
          <specular>0.8 0.8 0.8 1</specular>
        </material>
      </visual>
    </link>

    <!-- Joints -->
    <joint name="base_to_wheel" type="revolute">
      <parent>base_link</parent>
      <child>wheel_link</child>
      <pose>0 0.3 0 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-1e16</lower>
          <upper>1e16</upper>
          <effort>100</effort>
          <velocity>100</velocity>
        </limit>
      </axis>
    </joint>

    <link name="wheel_link">
      <pose>0 0.3 0 0 0 0</pose>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.01</iyy>
          <iyz>0</iyz>
          <izz>0.005</izz>
        </inertia>
      </inertial>

      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.1</length>
          </cylinder>
        </geometry>
      </collision>

      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.1</length>
          </cylinder>
        </geometry>
      </visual>
    </link>

    <!-- Gazebo-specific plugins -->
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/my_robot</robotNamespace>
    </plugin>
  </model>
</sdf>
```

## 1.2.4 Configuring Joint Properties for Simulation

### Joint Types in Gazebo
- **Fixed**: No degrees of freedom
- **Revolute**: Single axis rotation (hinges)
- **Prismatic**: Single axis translation (sliders)
- **Continuous**: Unlimited rotation (wheels)
- **Ball**: Three degrees of rotational freedom
- **Universal**: Two degrees of rotational freedom

### Joint Limits and Dynamics
```xml
<joint name="arm_joint" type="revolute">
  <parent link="upper_arm"/>
  <child link="lower_arm"/>
  <origin xyz="0 0 0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  <dynamics damping="0.1" friction="0.01"/>
</joint>
```

### Joint Safety Limits
For ROS2 integration, it's important to specify safety controllers:
```xml
<transmission name="tran1">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="arm_joint">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </joint>
  <actuator name="motor1">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

## 1.2.5 Robot State Publishers and TF Transforms

### Importance of Robot State Publisher
The robot state publisher is crucial for simulation as it:
- Publishes joint states to `/joint_states` topic
- Maintains TF tree for coordinate transformations
- Provides kinematic information for other ROS nodes

### Setting up Robot State Publisher
```xml
<!-- In your launch file -->
<node pkg="robot_state_publisher" exec="robot_state_publisher" name="robot_state_publisher">
  <param name="robot_description" value="$(var robot_description)"/>
  <param name="publish_frequency" value="50.0"/>
</node>
```

### Joint State Publisher for Control
For simulation, you'll also need joint state publisher:
```xml
<node pkg="joint_state_publisher" exec="joint_state_publisher" name="joint_state_publisher">
  <param name="rate" value="50"/>
  <param name="use_gui" value="false"/>
</node>
```

## 1.2.6 Spawning Robots in Gazebo

### Using spawn_entity Script
```bash
# Spawn a URDF robot
ros2 run gazebo_ros spawn_entity.py -entity my_robot -file /path/to/robot.urdf -x 0 -y 0 -z 1

# Spawn an SDF model
ros2 run gazebo_ros spawn_entity.py -entity my_robot -file /path/to/robot.sdf -x 0 -y 0 -z 1

# Spawn with specific initial pose
ros2 run gazebo_ros spawn_entity.py -entity my_robot -file /path/to/robot.urdf -x 1.0 -y 2.0 -z 0.0 -R 0.0 -P 0.0 -Y 1.57
```

### Spawning Through Code
```python
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity

class RobotSpawner(Node):
    def __init__(self):
        super().__init__('robot_spawner')
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def spawn_robot(self, robot_description, robot_name, initial_pose):
        req = SpawnEntity.Request()
        req.name = robot_name
        req.xml = robot_description
        req.initial_pose = initial_pose
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
```

## 1.2.7 Gazebo-Specific Plugins

### Gazebo ROS Control Plugin
The `gazebo_ros_control` plugin connects Gazebo to ROS2 control systems:
```xml
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/my_robot</robotNamespace>
    <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
  </plugin>
</gazebo>
```

### Sensor Plugins
Gazebo provides various sensor plugins for simulation:
```xml
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>600</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>camera_optical_frame</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## 1.2.8 Model Validation in Simulation

### Checking Model Behavior
After importing your robot model:
1. Verify all joints move as expected
2. Check that collision and visual geometries are properly aligned
3. Confirm that masses and inertias are realistic
4. Test that sensors provide appropriate data
5. Validate that the robot doesn't fall through the ground or behave unexpectedly

### Common Issues and Solutions
- **Robot falls through ground**: Check collision geometries and masses
- **Joints move unexpectedly**: Verify joint limits and dynamics
- **Sensors don't work**: Check plugin configuration and frame names
- **TF tree issues**: Verify robot state publisher setup and joint names

## Key Terms

- **URDF**: Unified Robot Description Format, the standard ROS format for robot models
- **SDF**: Simulation Description Format, Gazebo's native format
- **Link**: Rigid body component of a robot model
- **Joint**: Connection between links with specific degrees of freedom
- **Collision**: Geometry used for physics calculations
- **Visual**: Geometry used for rendering
- **Inertial**: Mass and inertia properties for physics simulation
- **Transmission**: Defines how joints connect to actuators
- **TF Tree**: Transform tree for coordinate system relationships
- **Robot State Publisher**: ROS node that publishes joint states and maintains TF tree

## Summary

This chapter covered the essential aspects of robot model integration in Gazebo, from understanding URDF and SDF formats to properly configuring joints and spawning robots. Proper model integration is crucial for realistic simulation and effective testing of robotics algorithms.

## Exercises

1. **URDF to SDF Conversion**: Create a simple robot model in URDF format and convert it to SDF using Gazebo tools. Compare the structures.

2. **Model Import**: Import a publicly available robot model (e.g., TurtleBot3, PR2) into Gazebo and verify its functionality.

3. **Joint Configuration**: Create a robot with multiple joint types (revolute, prismatic, continuous) and test their behavior in simulation.

4. **Sensor Integration**: Add a camera or LiDAR sensor to your robot model and verify that it publishes data in simulation.

5. **TF Validation**: Use `ros2 run tf2_tools view_frames` to visualize the TF tree of your robot in simulation and verify all transforms are correct.

## Next Steps

Continue to [1.3 Physics and Sensor Simulation](../week-3/physics-sensor-simulation.md) to learn about configuring physics properties and sensor models for realistic simulation.
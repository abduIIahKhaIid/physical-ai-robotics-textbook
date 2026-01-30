# Lab 1: Introduction to ROS 2 - Low GPU Variant

## Hardware Requirements

- ✅ CPU: Any modern processor (Intel i5 or equivalent)
- ✅ RAM: 8GB minimum (16GB recommended)
- ✅ GPU: Not required
- ✅ OS: Ubuntu 22.04 LTS (WSL2 on Windows also works)

## Overview

This lab variant is optimized for systems **without a dedicated GPU**. We'll use Gazebo Classic (CPU-based) instead of GPU-accelerated simulators.

## Prerequisites

```bash
# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# Install Gazebo Classic (CPU-based)
sudo apt install ros-humble-gazebo-ros-pkgs
```

## Part 1: Hello World Node

Create your first ROS 2 node that runs entirely on CPU.

```python
# hello_robot.py
import rclpy
from rclpy.node import Node

class HelloRobot(Node):
    def __init__(self):
        super().__init__('hello_robot')
        self.get_logger().info('Hello from ROS 2! (CPU-friendly version)')
        
        # Create a timer (no GPU needed)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
    def timer_callback(self):
        self.get_logger().info('Robot is alive!')

def main():
    rclpy.init()
    node = HelloRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Run it:**

```bash
python3 hello_robot.py
```

## Part 2: Simple Publisher-Subscriber

```python
# publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'robot_status', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Robot status: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main():
    rclpy.init()
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
```

## Part 3: Gazebo Classic Simulation (CPU-Only)

Launch a simple robot simulation without GPU requirements.

```xml
<!-- simple_robot.launch.py -->
<launch>
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
    <!-- CPU-friendly settings -->
    <arg name="physics" value="ode"/>
    <arg name="real_time_update_rate" value="10"/>  <!-- Reduced from 1000 -->
  </include>
</launch>
```

**Performance Tips for Low-End Hardware:**

1. **Reduce physics rate**: Use 10Hz instead of 1000Hz
2. **Disable shadows**: Set `<cast_shadows>false</cast_shadows>` in URDF
3. **Simple geometries**: Use basic shapes instead of meshes
4. **Headless mode**: Run without GUI for better performance

```bash
# Launch headless (no GUI)
ros2 launch gazebo_ros empty_world.launch.py gui:=false
```

## Cloud Alternative

Don't have good hardware? Use AWS RoboMaker:

```bash
# Install AWS CLI
pip install awscli

# Create simulation job (simplified)
aws robomaker create-simulation-job \
  --simulation-applications applicationName=MyRobot \
  --instance-type ml.t3.medium  # Free tier eligible
```

[See full AWS RoboMaker guide →](../cloud/aws-robomaker.md)

## What's Different from High-GPU Version?

| Feature | Low GPU (This Lab) | High GPU Version |
|---------|-------------------|------------------|
| Simulator | Gazebo Classic | Isaac Sim / Unity |
| Physics Rate | 10Hz | 100Hz+ |
| Rendering | CPU-based | RTX Ray Tracing |
| Sensors | Basic (Laser) | Full Suite (LiDAR, Depth Cameras) |
| Multi-Robot | Limited | Full Support |

## Next Steps

- [Lab 2: Topics and Services →](./lab2-low-gpu.md)
- [Install better hardware guide](../hardware/upgrade-guide.md)
- [Compare GPU vs CPU performance](../performance/comparison.md)

## Troubleshooting

**Issue**: Simulation is laggy

**Solution**: 
1. Reduce physics rate further: `real_time_update_rate: 5`
2. Use simpler robot models
3. Close other applications
4. Consider cloud deployment

**Issue**: Out of memory

**Solution**:
1. Close browser tabs
2. Increase swap space: `sudo fallocate -l 8G /swapfile`
3. Use headless mode
# Lab 1: Introduction to ROS 2 - Mid GPU Variant

## Hardware Requirements

- ✅ CPU: Intel i5-12600K or AMD Ryzen 7 5800X
- ✅ RAM: 16GB DDR4 (32GB recommended)
- ✅ GPU: **NVIDIA GTX 1660 / RTX 3060** (6-8GB VRAM)
- ✅ OS: Ubuntu 22.04 LTS
- ✅ Storage: 50GB free space

## Overview

This lab variant is optimized for **mid-range GPUs**. We'll use a hybrid approach: Gazebo for basic simulations and limited Isaac Sim usage for perception tasks.

## Prerequisites

```bash
# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# Install Gazebo (GPU-accelerated version)
sudo apt install ros-humble-gazebo-ros-pkgs

# Optional: Isaac Sim for specific tasks (if 8GB+ VRAM)
# See: https://developer.nvidia.com/isaac-sim
```

## Part 1: ROS 2 Basics (No GPU Required)

Start with standard ROS 2 concepts that work on any hardware.

```python
# minimal_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main():
    rclpy.init()
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Part 2: Gazebo Simulation with GPU Support

Use Gazebo with GPU rendering enabled (works well on mid-range GPUs).

```xml
<!-- robot.launch.py -->
<launch>
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    
    <!-- GPU-optimized settings -->
    <arg name="physics" value="ode"/>
    <arg name="real_time_update_rate" value="100"/>  <!-- Balanced -->
    <arg name="use_gpu_ray" value="true"/>  <!-- GPU for ray casting -->
  </include>
</launch>
```

**Performance Settings for Mid-Range GPU:**

```xml
<!-- In your world file -->
<world name="default">
  <physics type="ode">
    <max_step_size>0.01</max_step_size>  <!-- 100Hz -->
    <real_time_factor>1.0</real_time_factor>
    <real_time_update_rate>100</real_time_update_rate>
  </physics>
  
  <scene>
    <shadows>true</shadows>  <!-- Enabled, GPU can handle it -->
    <grid>false</grid>
  </scene>
</world>
```

## Part 3: Computer Vision with RealSense (GPU-Accelerated)

Use OpenCV with CUDA for faster image processing.

```python
# vision_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# If you have opencv-cuda:
# import cv2.cuda as cuda

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Image, '/processed_image', 10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # GPU-accelerated processing (if available)
        # gpu_img = cuda.GpuMat()
        # gpu_img.upload(cv_image)
        # processed = cuda.cvtColor(gpu_img, cv2.COLOR_BGR2GRAY)
        # result = processed.download()
        
        # CPU fallback
        result = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Publish
        out_msg = self.bridge.cv2_to_imgmsg(result, "mono8")
        self.publisher.publish(out_msg)

def main():
    rclpy.init()
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Part 4: Selective Isaac Sim Usage

For mid-range GPUs, use Isaac Sim only for specific tasks that benefit most from it.

### When to Use Isaac Sim (Mid-Range)

✅ **Good fit:**
- Synthetic data generation (run at lower res)
- Sensor simulation (cameras, LiDAR)
- Short perception tests

❌ **Avoid:**
- Long training runs (use Gazebo instead)
- Multi-robot scenarios (>2 robots)
- Real-time physics at 1kHz

### Optimized Isaac Sim Config

```python
# isaac_perception.py
from omni.isaac.kit import SimulationApp

# Mid-range GPU settings
simulation_app = SimulationApp({
    "headless": False,
    "renderer": "RayTracedLighting",
    "anti_aliasing": 1,  # Reduced from 3
    "width": 1280,       # Lower resolution
    "height": 720,
    "samples_per_pixel_per_frame": 8,  # Reduced from 64
})

from omni.isaac.core import World
from omni.isaac.sensor import Camera

world = World(
    physics_dt=1.0/60.0,   # 60Hz instead of 100Hz
    rendering_dt=1.0/30.0   # 30 FPS instead of 60
)

# Lower-res camera
camera = Camera(
    prim_path="/World/Camera",
    resolution=(640, 480),  # Lower res
    frequency=15  # 15 FPS instead of 30
)

world.reset()

for i in range(100):
    world.step(render=True)
    
    if i % 10 == 0:
        rgb = camera.get_rgba()
        print(f"Captured frame {i}")

simulation_app.close()
```

## Hybrid Workflow: Best of Both Worlds

Use Gazebo for development, Isaac Sim for final validation.

```bash
# Development (Fast iteration on mid-range GPU)
ros2 launch my_robot gazebo.launch.py

# Final testing (Photorealistic, slower)
python3 isaac_sim_test.py
```

## Performance Comparison

| Task | Gazebo (Mid-GPU) | Isaac Sim (Mid-GPU) | Notes |
|------|-----------------|-------------------|-------|
| Physics | 100Hz | 60Hz | Gazebo faster on mid-GPU |
| Rendering | 30-60 FPS | 15-30 FPS | Gazebo better |
| Multi-Robot | 2-3 robots | 1 robot | Gazebo more efficient |
| Perception | Basic | Photorealistic | Isaac Sim worth it here |
| Data Gen | Limited | Excellent | Use Isaac Sim |

## When to Upgrade to High GPU

Consider upgrading if:
- Isaac Sim runs <15 FPS consistently
- VRAM usage >90% (use `nvidia-smi`)
- You need >2 robots simultaneously
- Training takes >24 hours

**Recommended upgrade**: RTX 4070 Ti (12GB) or RTX 4080 (16GB)

## Cloud Alternative for Heavy Tasks

For occasional heavy workloads, use cloud:

```bash
# AWS EC2 with G5 instance (A10G GPU)
# $1.50/hour on-demand, $0.50/hour spot

# Google Cloud with T4 GPU
# $0.95/hour

# Only for:
# - Large-scale synthetic data generation
# - Multi-robot coordination (5+ robots)
# - Long RL training runs
```

[See Cloud Setup Guide →](../cloud/setup.md)

## Next Steps

- [Lab 2: Sensor Integration →](./lab2-mid-gpu.md)
- [Optimize Gazebo for your GPU](../optimization/gazebo-tuning.md)
- [GPU upgrade decision guide](../hardware/gpu-upgrade.md)

## Troubleshooting

**Issue**: Isaac Sim is very slow

**Solution**:
1. Reduce resolution: 640x480 or 800x600
2. Lower physics rate: 30-60Hz
3. Use Gazebo instead for this task
4. Consider cloud GPU for this specific lab

**Issue**: Gazebo uses 100% GPU

**Solution**:
1. Good! That means GPU acceleration is working
2. If overheating: reduce `real_time_update_rate` to 60
3. If other apps lag: close unnecessary programs

**Issue**: Out of VRAM

**Solution**:
1. Close Chrome/Firefox (uses 1-2GB VRAM)
2. Reduce Gazebo world complexity
3. Use headless mode: `gui:=false`
4. For Isaac Sim: use 480p resolution
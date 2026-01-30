# Lab 1: Introduction to ROS 2 - High GPU Variant

## Hardware Requirements

- ✅ CPU: Intel i7-13700K or AMD Ryzen 9 5900X+
- ✅ RAM: 32GB DDR5 (64GB recommended)
- ✅ GPU: **NVIDIA RTX 4070 Ti** (12GB VRAM) or better
  - RTX 4090 (24GB) recommended for full Isaac Sim
- ✅ OS: Ubuntu 22.04 LTS (native, not WSL2)
- ✅ Storage: 100GB+ SSD free space

## Overview

This lab variant leverages **high-end GPU acceleration** for photorealistic simulation, real-time physics, and AI-powered perception. You'll use NVIDIA Isaac Sim, the industry-standard platform for advanced robotics development.

## Prerequisites

### Install NVIDIA Isaac Sim

```bash
# Download Isaac Sim (requires NVIDIA account)
# Visit: https://developer.nvidia.com/isaac-sim

# Install Omniverse Launcher
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage
chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage

# Install Isaac Sim 2023.1.1 via Launcher
# Requires RTX GPU with latest drivers (535+)
```

### Verify GPU

```bash
nvidia-smi

# Should show:
# RTX 4090/4080/4070 Ti
# CUDA Version: 12.1+
# Memory: 12GB+ available
```

### Install ROS 2 Humble

```bash
sudo apt update
sudo apt install ros-humble-desktop-full
sudo apt install ros-humble-isaac-ros-*
```

## Part 1: Isaac Sim Hello World

Launch your first photorealistic robot simulation.

```python
# isaac_hello_world.py
from omni.isaac.kit import SimulationApp

# Launch with RTX enabled
simulation_app = SimulationApp({
    "headless": False,
    "renderer": "RayTracedLighting",  # RTX ray tracing
    "anti_aliasing": 3,
    "width": 1920,
    "height": 1080
})

from omni.isaac.core import World
from omni.isaac.core.robots import Robot
import numpy as np

# Create world with high-fidelity physics
world = World(
    stage_units_in_meters=1.0,
    physics_dt=1.0/100.0,  # 100Hz physics
    rendering_dt=1.0/60.0   # 60 FPS rendering
)

# Load Franka Emika Panda robot
from omni.isaac.franka import Franka
robot = world.scene.add(
    Franka(
        prim_path="/World/Franka",
        name="franka_robot",
        position=np.array([0, 0, 0])
    )
)

# Run simulation
world.reset()

for i in range(1000):
    world.step(render=True)
    
    if i % 100 == 0:
        print(f"Simulation step: {i}")

simulation_app.close()
```

**Run with GPU acceleration:**

```bash
python3 isaac_hello_world.py

# Expected: Photorealistic rendering at 60 FPS
# GPU utilization: 70-90%
```

## Part 2: ROS 2 + Isaac Sim Integration

Bridge Isaac Sim with ROS 2 for real-time control.

```python
# isaac_ros2_bridge.py
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
    "renderer": "RayTracedLighting"
})

# Enable ROS2 bridge
import omni.graph.core as og
from omni.isaac.core_nodes.scripts.utils import set_target_prims

# Create action graph for ROS2
keys = og.Controller.Keys
graph_path = "/World/ActionGraph"
(graph_handle, _, _, _) = og.Controller.edit({
    "graph_path": graph_path,
    "evaluator_name": "execution"
}, {
    og.Controller.Keys.CREATE_NODES: [
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
        ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
    ],
    og.Controller.Keys.CONNECT: [
        ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
    ],
})

# Robot will now publish to /joint_states and /clock
print("ROS2 bridge active. Subscribe with:")
print("ros2 topic echo /joint_states")

from omni.isaac.core import World
world = World()
world.reset()

while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
```

**Test ROS2 topics:**

```bash
# Terminal 1: Run Isaac Sim
python3 isaac_ros2_bridge.py

# Terminal 2: Check topics
ros2 topic list
ros2 topic echo /joint_states
ros2 topic hz /joint_states  # Should show 100Hz
```

## Part 3: Photorealistic Perception

Use Isaac Sim's synthetic data generation for training vision models.

```python
# synthetic_data_gen.py
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"renderer": "RayTracedLighting"})

from omni.isaac.core import World
from omni.isaac.sensor import Camera
import omni.replicator.core as rep

world = World()

# Create high-res RGB-D camera
camera = Camera(
    prim_path="/World/Camera",
    resolution=(1920, 1080),
    frequency=30
)

# Add depth and semantic segmentation
camera.initialize()
camera.add_distance_to_image_plane_to_frame()
camera.add_instance_segmentation_to_frame()

# Randomize scene for synthetic data
with rep.new_layer():
    # Randomize lighting
    rep.randomizer.light(
        temperature=rep.distribution.uniform(3000, 7000),
        intensity=rep.distribution.uniform(1000, 5000)
    )
    
    # Randomize materials
    with rep.trigger.on_frame(num_frames=100):
        rep.randomizer.materials(
            textures=rep.distribution.choice([
                "metal", "plastic", "wood", "concrete"
            ])
        )

# Run and capture
world.reset()

for frame in range(100):
    world.step(render=True)
    
    # Get annotated data
    rgb = camera.get_rgba()
    depth = camera.get_distance_to_image_plane()
    segmentation = camera.get_instance_segmentation()
    
    # Save for training (every 10 frames)
    if frame % 10 == 0:
        import numpy as np
        np.save(f"data/rgb_{frame}.npy", rgb)
        np.save(f"data/depth_{frame}.npy", depth)
        np.save(f"data/seg_{frame}.npy", segmentation)
        print(f"Saved frame {frame}")

simulation_app.close()
```

**Results**: 100 photorealistic, annotated training images in ~2 minutes

## Part 4: Multi-Robot Coordination

Simulate multiple robots simultaneously with GPU acceleration.

```python
# multi_robot_sim.py
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"renderer": "RayTracedLighting"})

from omni.isaac.core import World
from omni.isaac.wheeled_robots.robots import WheeledRobot
import numpy as np

world = World(physics_dt=1.0/100.0)

# Spawn 10 robots in formation
robots = []
for i in range(10):
    x = (i % 5) * 2.0
    y = (i // 5) * 2.0
    
    robot = WheeledRobot(
        prim_path=f"/World/Robot_{i}",
        name=f"robot_{i}",
        position=np.array([x, y, 0.1])
    )
    robots.append(robot)

world.reset()

# Coordinate movement
for step in range(1000):
    for i, robot in enumerate(robots):
        # Circle formation
        angle = (step + i * 36) * np.pi / 180
        vel_x = np.cos(angle) * 0.5
        vel_y = np.sin(angle) * 0.5
        robot.apply_wheel_actions([vel_x, vel_y])
    
    world.step(render=True)

simulation_app.close()
```

**Performance**: All 10 robots at 100Hz physics, 60 FPS rendering

**GPU utilization**: ~85% (RTX 4090)

## Optimization Tips for RTX GPUs

### 1. Enable Tensor Cores

```python
simulation_app = SimulationApp({
    "renderer": "RayTracedLighting",
    "dlss": True,  # NVIDIA DLSS upscaling
    "reflex": True,  # NVIDIA Reflex low latency
})
```

### 2. Maximize Physics Performance

```python
world = World(
    physics_dt=1.0/1000.0,  # 1kHz physics (requires RTX)
    rendering_dt=1.0/60.0,
    use_gpu_pipeline=True,   # PhysX GPU acceleration
    device="cuda:0"
)
```

### 3. Monitor Performance

```bash
# Watch GPU metrics
watch -n 1 nvidia-smi

# Profile with Nsight Systems
nsys profile -o isaac_profile python3 isaac_hello_world.py
```

## What You Get with High GPU

| Feature | High GPU (This Lab) | Low GPU Version |
|---------|-------------------|------------------|
| Renderer | RTX Ray Tracing | CPU Rasterization |
| Physics | 1000Hz GPU PhysX | 10Hz CPU ODE |
| Resolution | 1920x1080+ | 720p |
| FPS | 60 | 5-10 |
| Multi-Robot | 10+ robots | 1-2 robots |
| Synthetic Data | Photorealistic | Basic |
| Training Speed | 100x faster | Baseline |

## Advanced Projects

Now that you have the power, try these:

1. **Sim-to-Real Transfer**: Train in Isaac Sim, deploy to real robot
2. **Warehouse Automation**: Simulate 50 AMRs in a warehouse
3. **Digital Twin**: Create photorealistic twin of your lab
4. **Reinforcement Learning**: Train RL policies with Isaac Gym

[See Advanced Projects →](../advanced/projects.md)

## Next Steps

- [Lab 2: Advanced Perception with Isaac ROS →](./lab2-high-gpu.md)
- [Optimize Isaac Sim for your GPU](../optimization/isaac-tuning.md)
- [Enable remote rendering](../cloud/remote-streaming.md)

## Troubleshooting

**Issue**: Low FPS despite high-end GPU

**Solution**:
1. Update NVIDIA drivers: `sudo ubuntu-drivers install`
2. Enable DLSS: `dlss=True` in SimulationApp
3. Check GPU isn't thermal throttling: `nvidia-smi dmon`

**Issue**: Out of VRAM

**Solution**:
1. Reduce resolution: `"height": 720, "width": 1280`
2. Use LOD (Level of Detail) for distant objects
3. Enable texture streaming
4. Upgrade to RTX 4090 (24GB VRAM)
---
title: "1.1 Gazebo Simulation Fundamentals"
description: "Introduction to Gazebo simulation environment and core concepts for robotics development."
tags: [gazebo, simulation, fundamentals, robotics, environment]
learning-objectives:
  - "Understand the core concepts of Gazebo simulation environment"
  - "Set up and configure a basic Gazebo simulation environment"
  - "Create and modify simple world files"
  - "Spawn basic objects and robots in simulation"
  - "Navigate the Gazebo interface and controls"
---

# 1.1 Gazebo Simulation Fundamentals

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the core concepts of Gazebo simulation environment
- Set up and configure a basic Gazebo simulation environment
- Create and modify simple world files
- Spawn basic objects and robots in simulation
- Navigate the Gazebo interface and controls

## Introduction

Gazebo is a powerful, open-source robotics simulator that provides realistic physics simulation, high-quality graphics, and convenient programmatic interfaces. It enables researchers and developers to test algorithms, train robots, and conduct experiments in a safe, cost-effective virtual environment before deploying to real hardware.

This chapter introduces you to the fundamental concepts of Gazebo simulation and provides hands-on experience with the simulation environment.

## 1.1.1 What is Gazebo?

Gazebo is a 3D dynamic simulator with accurate physics simulation that mimics real-world conditions. It provides:

### Core Features
- **Physics Engine**: Multiple physics engines (ODE, Bullet, Simbody) for accurate simulation
- **Rendering**: High-quality graphics rendering with support for lighting, shadows, and textures
- **Sensors**: Realistic simulation of various sensors (cameras, lidars, IMUs, etc.)
- **Plugins**: Extensible architecture supporting custom plugins for specific functionality
- **ROS Integration**: Seamless integration with ROS and ROS2 for robotics development

### Use Cases
- Testing control algorithms in a safe environment
- Training machine learning models
- Prototyping robot designs
- Validating sensor configurations
- Educational purposes

## 1.1.2 Gazebo Architecture

### Main Components
- **Gazebo Server**: Core simulation engine that handles physics, rendering, and communications
- **Gazebo Client**: Graphical user interface for interacting with the simulation
- **Gazebo Transport**: Messaging system for inter-process communication
- **Gazebo Plugins**: Extensible modules that add functionality to the simulation

### Communication Framework
Gazebo uses a publish-subscribe messaging system similar to ROS topics. The transport layer handles communication between the server, clients, and plugins.

## 1.1.3 Setting Up Gazebo Environment

### Installation Requirements
- Ubuntu 20.04 or 22.04
- ROS2 Humble Hawksbill or newer
- Minimum 8GB RAM (16GB+ recommended)
- OpenGL 2.1 compatible graphics card
- At least 10GB free disk space

### Installation Steps
```bash
# Install Gazebo Garden (recommended version)
sudo apt update
sudo apt install gazebo libgazebo-dev

# Or install via ROS2 packages
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-gazebo-plugins
sudo apt install ros-humble-gazebo-dev
```

### Verification
```bash
# Check Gazebo version
gz --version

# Launch Gazebo server (in headless mode)
gz sim -s

# Launch Gazebo GUI client
gz sim gui
```

## 1.1.4 Gazebo Interface and Controls

### Main Interface Components
- **Scene Window**: 3D visualization of the simulation environment
- **Time Panel**: Simulation time, real-time factor, and pause/play controls
- **Model List**: Hierarchical view of all objects in the scene
- **Layers Panel**: Control visibility of different elements (lights, contacts, etc.)
- **Tools Menu**: Access to various simulation tools and plugins

### Navigation Controls
- **Orbit**: Right-click and drag to orbit around the scene
- **Pan**: Middle-click and drag to pan the camera
- **Zoom**: Scroll wheel to zoom in/out
- **Focus**: Double-click on an object to center the camera on it

## 1.1.5 World Files and Structure

### World File Format
Gazebo uses SDF (Simulation Description Format) to define simulation environments. SDF is an XML-based format that describes:
- Models (robots, objects, etc.)
- Physics properties
- Scene lighting and environment
- Plugins and sensors

### Basic World Structure
```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="my_world">
    <!-- Physics engine configuration -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>

    <!-- Environment lighting -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.6 0.2 -0.8</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Your models go here -->
  </world>
</sdf>
```

## 1.1.6 Spawning Objects and Robots

### Using spawn_entity Script
Gazebo provides a command-line tool to spawn models into the simulation:

```bash
# Spawn a basic box
gz model -f box.sdf -m box_model

# Spawn a predefined model from Gazebo Model Database
gz model -m unit_box -f /usr/share/gazebo-11/models/box/model.sdf

# Spawn with specific position and orientation
gz model -m my_robot -f robot.urdf -x 1.0 -y 2.0 -z 0.0 -R 0 -P 0 -Y 1.57
```

### Model Spawning Parameters
- `-m` or `--name`: Model name in the simulation
- `-f` or `--file`: Path to the model file (URDF, SDF, etc.)
- `-x`, `-y`, `-z`: Position coordinates
- `-R`, `-P`, `-Y`: Roll, pitch, yaw orientation angles

## 1.1.7 Running Your First Simulation

### Creating a Simple World
Create a file named `simple_world.sdf`:

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="simple_world">
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>

    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.6 0.2 -0.8</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </visual>
      </link>
    </model>

    <model name="unit_box">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Running the Simulation
```bash
# Launch Gazebo with your world
gz sim simple_world.sdf

# Or run in headless mode (without GUI)
gz sim -r simple_world.sdf
```

## Key Terms

- **SDF (Simulation Description Format)**: XML-based format for describing simulation environments
- **Gazebo Server**: Core simulation engine that handles physics and rendering
- **Gazebo Client**: Graphical interface for interacting with the simulation
- **Physics Engine**: Software that calculates physical interactions in the simulation
- **Model**: Any object in the simulation (robot, obstacle, environment item)
- **Plugin**: Extensible module that adds functionality to the simulation
- **Real-time Factor**: Ratio of simulation time to real-world time
- **Collision**: Geometry used for physics calculations
- **Visual**: Geometry used for rendering

## Summary

This chapter introduced you to the fundamentals of Gazebo simulation. You learned about the architecture of Gazebo, how to set up the environment, navigate the interface, and create basic simulations. Understanding these concepts is crucial for effectively using Gazebo in robotics development.

## Exercises

1. **Basic Setup**: Install Gazebo on your system and verify the installation by launching the GUI.

2. **World Creation**: Create a simple world file with a ground plane and two different colored boxes positioned at different locations.

3. **Model Spawning**: Create a simple SDF model of a cylinder and spawn it in a running Gazebo simulation using the command line.

4. **Interface Navigation**: Practice navigating the Gazebo interface, including camera controls, pausing/resuming simulation, and inspecting model properties.

5. **Physics Parameters**: Experiment with different physics parameters (step size, real-time factor) and observe their effects on simulation behavior.

## Next Steps

Continue to [1.2 Robot Model Integration](../week-2/robot-model-integration.md) to learn how to import and configure robot models in Gazebo.
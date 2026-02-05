---
title: "Lab 1: Isaac Sim Basics"
description: "Hands-on lab exercise to practice Isaac Sim fundamentals"
tags: [lab, isaac-sim, basics, hands-on, practical]
sidebar_position: 3
---

# Lab 1: Isaac Sim Basics

## Learning Objectives

After completing this lab, you will be able to:
- Launch Isaac Sim and navigate the interface
- Import a robot model into a scene
- Configure basic sensors (camera and LiDAR)
- Run a simple simulation and observe robot behavior
- Verify that Isaac Sim is properly set up for the module

## Prerequisites

Before starting this lab, you must:
- Complete the Introduction to Isaac Sim Environment lesson
- Complete the Isaac Sim Setup and Basic Operations lesson
- Have Isaac Sim installed and configured on your system
- Have access to a compatible GPU with sufficient VRAM

## Equipment and Software Required

- NVIDIA Isaac Sim installed and activated
- Compatible GPU (recommended: RTX 2060 or better with 6GB+ VRAM)
- Computer with at least 16GB RAM
- Internet connection for initial asset downloads

## Estimated Time

This lab should take approximately 60-90 minutes to complete.

## Pre-Lab Setup

1. Ensure Isaac Sim is properly installed and can launch without errors
2. Verify your GPU drivers are up to date
3. Make sure you have internet connectivity for downloading assets (first run only)
4. Close other GPU-intensive applications to ensure adequate performance

## Lab Procedure

### Step 1: Launch Isaac Sim and Verify Installation

1. Open Omniverse Launcher
2. Launch Isaac Sim from the app library
3. Wait for Isaac Sim to initialize (this may take 1-2 minutes on first launch)
4. Verify the interface loads correctly with the default scene
5. Check the console for any error messages

### Step 2: Load a Basic Scene

1. Go to File → Open Scene
2. Navigate to the Isaac Sim examples directory
3. Select the "Simple Room" scene
4. Wait for the scene to load completely
5. Use the navigation controls to explore the scene:
   - Orbit: Right-click and drag
   - Pan: Middle-click and drag
   - Zoom: Scroll wheel

### Step 3: Import a Robot Model

1. With the Simple Room scene loaded, go to Isaac Sim → Create → Robot
2. Select the "TurtleBot3 Burger" model (or another simple robot if available)
3. Position the robot in the center of the room using the transform tools
4. Verify the robot appears correctly in the scene
5. Check that the robot hierarchy is visible in the Stage panel

### Step 4: Configure Basic Sensors

1. Select the robot in the Stage panel
2. Add an RGB camera:
   - Go to Isaac Sim → Create → Sensor → RGB Camera
   - Position the camera on the robot's front, slightly above the base
   - Set resolution to 640x480
   - Set focal length to 64.0
3. Add a LiDAR sensor:
   - Go to Isaac Sim → Create → Sensor → LiDAR
   - Position the LiDAR on the robot's front, at base level
   - Set range to 0.1m to 10.0m
   - Set channels to 64
4. Verify both sensors appear correctly positioned on the robot

### Step 5: Configure Simulation Settings

1. Open the Settings window (Window → Settings)
2. Navigate to Physics settings
3. Set Gravity to [0.0, 0.0, -9.81] (Earth-like gravity)
4. Set Solver to "TGS" (Truncated Gauss-Seidel)
5. Set Substeps to 1 for this basic simulation
6. Navigate to Streaming settings
7. Set Render Resolution to 1280x720 (or adjust based on performance)
8. Close the Settings window

### Step 6: Run the Simulation

1. Click the Play button in the timeline to start the simulation
2. Observe the robot in the viewport - it should remain stationary initially
3. Use the Isaac Sim scripting interface or ROS bridge (if configured) to send basic commands:
   - Send a low-speed forward velocity command
   - Observe the robot moving forward in the scene
   - Send a rotation command
   - Observe the robot rotating in place
4. Pause the simulation after observing basic movement

### Step 7: Check Sensor Data

1. With the simulation running, verify that sensors are publishing data:
   - Check the Isaac Sim console for sensor messages
   - If using ROS bridge, verify camera and LiDAR topics are active
2. If possible, visualize the sensor data in a separate window or tool
3. Take screenshots of the robot with its sensors in the scene

### Step 8: Save Your Work

1. Go to File → Save As
2. Create a new folder for this lab in your working directory
3. Save the scene as "lab1_burger_bot.usd"
4. Verify the file saves without errors

## Expected Results

During this lab, you should observe:
- Isaac Sim launches without errors and loads the Simple Room scene
- The TurtleBot3 model imports successfully and appears correctly in the scene
- Both camera and LiDAR sensors are properly positioned on the robot
- The robot responds to movement commands during simulation
- Sensor data is being generated and published
- The scene saves successfully to your local directory

## Verification Steps

To verify your lab was successful:

1. Isaac Sim launches and loads scenes without errors
2. Robot model is properly imported and positioned in the scene
3. Both camera and LiDAR sensors are correctly configured and visible
4. Robot responds to movement commands during simulation
5. Scene saves successfully to your local directory
6. All sensors are publishing data (verified through console or external tools)

## Troubleshooting

Common issues and solutions:

- **Isaac Sim won't launch**: Check GPU drivers and system requirements. Ensure Omniverse Launcher is logged in with a valid NVIDIA account.
- **Robot model not appearing**: Verify the robot was properly selected from the Isaac Sim robot library. Check the Stage panel to confirm the robot hierarchy exists.
- **Sensors not showing**: Make sure sensors were properly attached to the robot. Check that the sensor properties are correctly configured.
- **Robot not responding to commands**: If using ROS bridge, verify the bridge is properly configured and running. Check topic names and connections.
- **Poor performance**: Reduce graphics settings in Isaac Sim. Close other GPU-intensive applications. Consider lowering simulation frequency.

## Lab Questions

Answer these questions to confirm your understanding:

1. What are the minimum hardware requirements for running Isaac Sim effectively?
2. How do you add a camera sensor to a robot in Isaac Sim?
3. What is the purpose of the Stage panel in Isaac Sim?
4. How can you verify that sensors are properly publishing data during simulation?

## Conclusion

In this lab, you successfully set up Isaac Sim, imported a robot model, configured basic sensors, and ran a simple simulation. You verified that your installation is working correctly and gained hands-on experience with the fundamental operations needed for the rest of this module. These skills will be essential as you progress to more complex perception and navigation tasks in upcoming weeks.

## References

- [Isaac Sim User Guide](https://docs.omniverse.nvidia.com/isaacsim/latest/user_guide.html)
- [Isaac Sim Robot Import Tutorial](https://docs.omniverse.nvidia.com/isaacsim/latest/user_guide/advanced_tutorials/tutorial_2_robot_import.html)
- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/)

:::caution
Always save your work frequently during Isaac Sim sessions to prevent loss of progress.
:::

:::tip
Take screenshots of your working scene for your portfolio or assignment submission.
:::
---
title: "Week 4 Exercises: Gazebo-ROS2 Integration"
description: "Practical exercises to reinforce understanding of Gazebo-ROS2 integration for hardware-in-the-loop testing."
tags: [ros2, integration, gazebo, plugins, simulation, exercises, robotics]
learning-objectives:
  - "Connect ROS2 nodes to Gazebo for hardware-in-the-loop testing"
  - "Configure Gazebo plugins for ROS2 communication"
  - "Implement robot control systems in simulation"
  - "Validate control algorithms in simulation environment"
---

# Week 4 Exercises: Gazebo-ROS2 Integration

## Learning Objectives

By completing these exercises, you will:
- Connect ROS2 nodes to Gazebo for hardware-in-the-loop testing
- Configure Gazebo plugins for ROS2 communication
- Implement robot control systems in simulation
- Validate control algorithms in simulation environment
- Understand the benefits and limitations of simulation-based development

## Exercise 1: Basic Gazebo-ROS2 Connection

### Task
Establish a basic connection between Gazebo and ROS2, verifying that messages can flow between them.

### Setup
- Gazebo installation with ROS2 plugins
- ROS2 environment (Humble Hawksbill or newer)
- Basic robot model in Gazebo

### Instructions
1. Launch Gazebo with a simple robot model that has ROS2 plugins configured
2. Verify that the robot model is recognized by ROS2 (check topics and services)
3. Confirm that joint states are being published to `/joint_states` topic
4. Verify that TF tree is being published with `ros2 run tf2_tools view_frames`
5. Test basic communication by sending a simple command to the robot
6. Confirm that sensor data is being published from Gazebo to ROS2
7. Document the connection process and verification steps

### Expected Output
- Gazebo running with robot model
- Joint states published to `/joint_states` topic
- TF tree properly published and viewable
- Basic commands successfully sent to robot
- Sensor data published from simulation to ROS2

### Verification
- Check available topics with `ros2 topic list`
- Echo joint states with `ros2 topic echo /joint_states`
- Verify TF tree with `ros2 run tf2_tools view_frames`
- Confirm sensor topics are publishing data
- Verify command topics accept messages

### Troubleshooting
- If joint states aren't published, check robot_state_publisher and plugins
- If TF tree is incomplete, verify robot_description and joint names
- If command topics don't respond, check controller configuration
- If sensor data isn't available, verify sensor plugin configuration

---

## Exercise 2: Controller Manager Setup and Configuration

### Task
Configure and test the ROS2 controller manager with Gazebo for robot control.

### Setup
- Robot model with controllable joints
- ROS2 with controller manager
- Gazebo with gazebo_ros2_control plugin

### Instructions
1. Create a controller configuration file for your robot (position, velocity, or effort controllers)
2. Configure the gazebo_ros2_control plugin in your robot's SDF/URDF
3. Launch Gazebo with the robot and controller manager
4. Load and activate the controllers using ros2 control commands
5. Test commanding joints through the controller interfaces
6. Verify that commands are properly transmitted to Gazebo
7. Test different controller types (position, velocity, effort) and compare behavior
8. Document the controller configuration and testing process

### Expected Output
- Controller manager running and managing robot controllers
- Controllers loaded and activated successfully
- Joint commands successfully transmitted to Gazebo
- Different controller types behave as expected
- Control commands produce appropriate robot movements

### Verification
- Check active controllers with `ros2 control list_controllers`
- Verify controller interfaces with `ros2 control list_hardware_interfaces`
- Confirm command topics are accepting messages
- Verify that joint movements match commanded values
- Test that controllers switch properly between states

### Troubleshooting
- If controllers won't load, check controller configuration file syntax
- If commands don't reach Gazebo, verify plugin and controller configuration
- If joints don't respond, check joint names match between URDF and controller config
- If controllers fail to activate, verify hardware interface compatibility

---

## Exercise 3: Sensor Data Processing Pipeline

### Task
Create a complete sensor data processing pipeline from Gazebo sensors to ROS2 nodes.

### Setup
- Robot with multiple sensor types (camera, LiDAR, IMU)
- ROS2 environment for processing
- Appropriate sensor processing nodes

### Instructions
1. Configure your robot with multiple sensor types (camera, LiDAR, IMU)
2. Ensure all sensors are properly publishing data to ROS2 topics
3. Create a simple ROS2 node that subscribes to sensor data
4. Process the sensor data (e.g., convert camera image, analyze LiDAR scan, interpret IMU data)
5. Publish processed results to new topics
6. Visualize the processed data using rviz2
7. Test the complete pipeline from sensor simulation to data processing
8. Document the pipeline architecture and data flow

### Expected Output
- Multiple sensor types publishing data from Gazebo
- ROS2 node successfully subscribing to sensor data
- Processed data published to new topics
- Data visualization working in rviz2
- Complete pipeline functioning end-to-end

### Verification
- Check that all sensor topics are publishing data
- Verify that your processing node receives data correctly
- Confirm processed data is published as expected
- Test that visualization displays processed data properly
- Validate that data rates and formats are appropriate

### Troubleshooting
- If sensor topics aren't publishing, check plugin configuration
- If data isn't received, verify topic names and message types
- If processing fails, check message type compatibility and data formats
- If visualization doesn't work, verify coordinate frames and message types

---

## Exercise 4: Robot Control Implementation

### Task
Implement a complete robot control system that operates in the Gazebo simulation environment.

### Setup
- Mobile robot model in Gazebo
- ROS2 control stack
- Navigation or manipulation task requirements

### Instructions
1. Design a control system for a specific task (navigation, manipulation, etc.)
2. Implement the controller as a ROS2 node that subscribes to sensor data
3. Have the controller publish commands to the robot's actuator interfaces
4. Test the control system in Gazebo with various scenarios
5. Implement safety checks and error handling in your controller
6. Add logging and debugging capabilities to your controller
7. Evaluate the controller's performance in simulation
8. Document the control architecture and implementation details

### Expected Output
- Working control system that operates the simulated robot
- Controller node that processes sensor data and sends commands
- Safety checks and error handling implemented
- Performance evaluation of the control system
- Documentation of control architecture and implementation

### Verification
- Robot responds appropriately to controller commands
- Control system handles various scenarios successfully
- Safety checks prevent unsafe robot behavior
- Performance metrics meet expectations
- Controller operates reliably over extended periods

### Troubleshooting
- If robot doesn't respond to commands, check controller configuration
- If control is unstable, tune control parameters
- If safety checks are too restrictive, adjust safety parameters
- If performance is poor, optimize controller algorithms

---

## Exercise 5: Plugin Development for Custom Functionality

### Task
Develop a custom Gazebo plugin to extend functionality and integrate with ROS2.

### Setup
- Gazebo development environment
- ROS2 development environment
- C++ build tools (g++, cmake)

### Instructions
1. Identify a custom functionality needed for your robot (e.g., custom sensor, actuator, or controller)
2. Create a new Gazebo plugin that implements this functionality
3. Integrate the plugin with ROS2 communication (publishing/subscribing to topics)
4. Build the plugin and test it in Gazebo
5. Verify that the plugin communicates properly with ROS2
6. Test the plugin under various conditions and scenarios
7. Document the plugin architecture and usage
8. Add proper error handling and logging to the plugin

### Expected Output
- Custom Gazebo plugin successfully built and loaded
- Plugin communicates with ROS2 topics/services
- Custom functionality works as expected in simulation
- Plugin handles errors gracefully
- Documentation for using the custom plugin

### Verification
- Plugin loads without errors in Gazebo
- Plugin communicates properly with ROS2
- Custom functionality works as intended
- Plugin handles edge cases and errors appropriately
- Performance is acceptable for real-time simulation

### Troubleshooting
- If plugin won't compile, check dependencies and build configuration
- If plugin doesn't load, verify plugin filename and Gazebo version compatibility
- If ROS2 communication fails, check topic/service names and message types
- If performance is poor, optimize plugin algorithms and callbacks

---

## Exercise 6: Hardware-in-the-Loop Testing Scenario

### Task
Design and implement a complete hardware-in-the-loop testing scenario that demonstrates the benefits of simulation-based development.

### Setup
- Robot model with realistic sensors and actuators
- Complete ROS2 system with navigation or manipulation stack
- Various testing scenarios and environments

### Instructions
1. Design a realistic testing scenario that would benefit from simulation (e.g., navigation through unknown environment)
2. Create a Gazebo world that represents the testing environment
3. Implement the complete ROS2 system (navigation, perception, planning, control)
4. Run the complete system in simulation for the designed scenario
5. Collect performance metrics and data during the simulation run
6. Analyze the results and compare with expected real-world performance
7. Document the advantages of simulation-based testing in this scenario
8. Identify potential limitations and how to address them

### Expected Output
- Complete hardware-in-the-loop testing scenario
- Working ROS2 system operating in simulation
- Performance metrics and analysis results
- Documentation of simulation benefits and limitations
- Recommendations for bridging simulation-to-reality gap

### Verification
- Complete system operates successfully in simulation
- Performance metrics are collected and analyzed
- Simulation results are consistent and reproducible
- Benefits of simulation-based testing are clearly demonstrated
- Limitations are properly identified and documented

### Troubleshooting
- If system is too complex, simplify the scenario or break into smaller components
- If simulation is unstable, check physics parameters and robot configuration
- If performance metrics aren't collected properly, verify logging and monitoring setup
- If results aren't reproducible, ensure deterministic simulation conditions

---

## Troubleshooting Tips

- **Connection Issues**: Verify that Gazebo plugins are properly loaded and ROS2 is sourced
- **Controller Problems**: Check controller configuration files and hardware interface compatibility
- **Sensor Data Issues**: Confirm plugin configuration and topic/message type compatibility
- **Performance Problems**: Monitor resource usage and optimize plugin/controller code
- **Integration Failures**: Use ROS2 command-line tools to debug topic/service connections

## Challenge Extension

For advanced students: Create a complete multi-robot simulation with:
- Coordination and communication between multiple robots
- Shared perception and mapping systems
- Distributed control algorithms
- Competition or collaboration scenarios
- Realistic communication delays and failures
- Comprehensive performance evaluation and analysis

---

## Summary

These exercises provided hands-on experience with Gazebo-ROS2 integration, covering basic connections, controller setup, sensor processing, control implementation, custom plugin development, and complete hardware-in-the-loop scenarios. This integration is crucial for effective simulation-based robotics development.

## Next Steps

After completing these exercises, continue to [Module 3: Control Systems and Locomotion](../../module-3/) to learn about feedback control systems and locomotion algorithms for humanoid robots.
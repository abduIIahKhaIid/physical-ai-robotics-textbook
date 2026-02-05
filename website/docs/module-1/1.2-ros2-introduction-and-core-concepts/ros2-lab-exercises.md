---
title: "Week 2 Exercises: ROS2 Introduction and Core Concepts"
description: "Practical exercises to implement and understand ROS2 concepts for Physical AI systems."
tags: [ros2, exercises, robotics, implementation, physical-ai]
learning-objectives:
  - "Implement ROS2 nodes for robotics applications"
  - "Design appropriate communication patterns for Physical AI"
  - "Configure Quality of Service settings for different data types"
  - "Create custom message types for specific applications"
---

# Week 2 Exercises: ROS2 Introduction and Core Concepts

## Learning Objectives

By completing these exercises, you will:
- Implement ROS2 nodes for robotics applications
- Design appropriate communication patterns for Physical AI systems
- Configure Quality of Service settings for different data types
- Create custom message types for specific robotics applications
- Understand the practical challenges of ROS2 implementation

## Exercise 1: Basic ROS2 Node Implementation

### Task
Create a simple ROS2 publisher and subscriber pair to understand the basic communication patterns.

### Setup
- Install ROS2 Humble Hawksbill (or newer LTS version)
- Create a new ROS2 package called `physical_ai_examples`
- Use either Python or C++ for implementation

### Instructions
1. Create a publisher node that publishes a counter value every second
2. Create a subscriber node that receives and logs the counter values
3. Use the `std_msgs/Int32` message type
4. Set up appropriate QoS settings for reliable communication
5. Test the communication between nodes

### Expected Output
- Publisher node running and publishing messages
- Subscriber node receiving and logging messages
- Both nodes communicating successfully

### Verification
- Messages are published and received without loss
- Nodes can be started independently
- Proper logging of received messages

---

## Exercise 2: Sensor Data Simulation

### Task
Implement a simulated sensor node that publishes realistic sensor data and a processing node that interprets it.

### Setup
- Use the `sensor_msgs/LaserScan` message type
- Create a simulated laser scanner that publishes 360-degree data
- Create a simple obstacle detection algorithm

### Instructions
1. Implement a laser scanner simulator that publishes `LaserScan` messages
2. Set appropriate QoS for sensor data (consider frequency and reliability needs)
3. Create an obstacle detector that subscribes to laser data
4. Have the detector publish obstacle distance information
5. Visualize the data using RViz2

### Expected Output
- Laser scan data published at regular intervals
- Obstacle detection working in real-time
- Visualization showing both raw and processed data

### Verification
- Laser scan data looks realistic (ranges within expected bounds)
- Obstacle detection works correctly
- Appropriate QoS settings for sensor data

---

## Exercise 3: Custom Message Creation

### Task
Design and implement custom message types for a specific Physical AI application.

### Setup
- Design message types for humanoid robot joint control
- Consider safety limits and status information

### Instructions
1. Design a custom message for humanoid joint commands that includes:
   - Joint names
   - Position, velocity, and effort targets
   - Safety limits and tolerances
2. Create the `.msg` file in your package
3. Implement a node that publishes these custom messages
4. Implement a node that subscribes and validates the commands
5. Test the custom message communication

### Expected Output
- Custom message definition files created
- Publisher and subscriber nodes working with custom messages
- Proper validation of safety limits

### Verification
- Custom messages compile successfully
- Communication works between nodes
- Safety limits are properly validated

---

## Exercise 4: Service Implementation

### Task
Create a ROS2 service for inverse kinematics computation.

### Setup
- Use a simple 2D arm model for demonstration
- Create appropriate request and response message types

### Instructions
1. Define service message types for IK computation:
   - Request: target position (x, y)
   - Response: joint angles or failure indication
2. Implement the IK service server
3. Implement a client that calls the service periodically
4. Handle error cases where IK solution is not possible
5. Test with various target positions

### Expected Output
- IK service running and responding to requests
- Client successfully calling the service
- Proper error handling for unreachable positions

### Verification
- Service responds correctly to valid requests
- Error handling works for invalid requests
- Response times are reasonable for real-time use

---

## Exercise 5: Quality of Service Experimentation

### Task
Experiment with different QoS settings to understand their impact on communication.

### Setup
- Use the publisher/subscriber from Exercise 1
- Test different QoS profiles

### Instructions
1. Modify your publisher to use different QoS settings:
   - Reliable vs Best Effort
   - Keep Last vs Keep All history
   - Different queue depths
2. Measure the impact on message delivery
3. Create a high-frequency publisher to stress-test communication
4. Monitor for message loss and latency
5. Document findings about appropriate QoS for different use cases

### Expected Output
- Different QoS configurations tested
- Measurements of message delivery rates
- Understanding of QoS impact on performance

### Verification
- QoS settings properly configured
- Measurements taken under different conditions
- Clear understanding of when to use different QoS settings

---

## Exercise 6: Launch File Configuration

### Task
Create comprehensive launch files to start your ROS2 system with proper configuration.

### Setup
- Use all nodes created in previous exercises
- Create parameter files for configuration

### Instructions
1. Create a launch file that starts:
   - Laser scanner simulator
   - Obstacle detector
   - Any other nodes from previous exercises
2. Include parameter configuration for each node
3. Add conditional startup based on arguments
4. Create a master launch file that can start the entire system
5. Test the launch system

### Expected Output
- Launch files created and working
- System starts with proper configuration
- Parameters loaded correctly

### Verification
- All nodes start correctly via launch files
- Parameters are properly loaded
- System behaves as expected when launched

---

## Exercise 7: Physical AI Integration Challenge

### Task
Combine all concepts learned to create a simple Physical AI behavior.

### Setup
- Integrate sensor processing, decision making, and actuation
- Create a simple behavior like obstacle avoidance

### Instructions
1. Combine nodes from previous exercises to create:
   - Sensor processing (laser scan → obstacles)
   - Decision making (obstacles → navigation commands)
   - Simple actuation (commands → motion simulation)
2. Use appropriate message types and QoS settings
3. Implement safety checks and error handling
4. Test the complete behavior
5. Document the system architecture

### Expected Output
- Complete behavior system working
- Proper integration of all components
- Safety and error handling implemented

### Verification
- System responds appropriately to sensor input
- Safety limits are respected
- All components communicate correctly

---

## Troubleshooting Tips

- **Node Communication Issues**: Check that nodes have the correct names and topics match exactly
- **Build Problems**: Ensure all dependencies are properly declared in package.xml and CMakeLists.txt
- **QoS Issues**: Match QoS settings between publishers and subscribers
- **Performance Problems**: Use appropriate QoS settings for your use case
- **Parameter Loading**: Verify parameter file paths and syntax

## Challenge Extension

For advanced students: Deploy your ROS2 system to a physical robot platform (real or simulated) and test the behavior in a realistic environment. Consider additional challenges like:

- Network communication between different machines
- Real sensor noise and uncertainty
- Real-time performance requirements
- Safety system integration

---

## Summary

These exercises provided hands-on experience with ROS2 core concepts including nodes, messages, services, QoS settings, and system composition. Through practical implementation, you gained understanding of the challenges and solutions involved in creating ROS2-based Physical AI systems.

## Next Steps

After completing these exercises, continue to [Week 2 Quiz](./quiz.md) to test your understanding of ROS2 concepts and implementation.
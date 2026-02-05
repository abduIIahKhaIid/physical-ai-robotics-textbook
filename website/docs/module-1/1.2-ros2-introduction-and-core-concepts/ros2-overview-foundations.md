---
title: "1.2 Introduction to ROS2"
description: "Learn about the Robot Operating System version 2, its architecture, and core concepts for robotics development."
tags: [ros2, robotics, middleware, nodes, topics, services]
learning-objectives:
  - "Understand the purpose and architecture of ROS2"
  - "Identify and explain ROS2 core concepts: nodes, topics, services"
  - "Describe the role of ROS2 in robotics development"
---

# 1.2 Introduction to ROS2

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the purpose and architecture of ROS2
- Identify and explain ROS2 core concepts: nodes, topics, services, parameters
- Describe the role of ROS2 in robotics development
- Compare ROS2 with traditional software frameworks for robotics
- Recognize the advantages of using ROS2 for Physical AI systems

## Introduction

The Robot Operating System 2 (ROS2) is not an operating system in the traditional sense, but rather a flexible framework for writing robot software. It provides libraries, tools, and conventions that simplify the development of complex robotic systems. ROS2 represents a significant evolution from ROS1, addressing critical issues like real-time performance, security, and scalability.

For Physical AI systems, ROS2 serves as the backbone that enables different components of a robot to communicate effectively. Whether it's a camera providing visual input, a controller managing motor commands, or a navigation system planning paths, ROS2 facilitates seamless interaction between these components.

## 1.2.1 What is ROS2?

### Purpose and Motivation

ROS2 was developed to address the growing needs of the robotics community. As robots became more complex and applications more diverse, the limitations of ROS1 became apparent. ROS2 was designed with the following goals:

- **Real-time performance**: Support for time-critical applications where delays are unacceptable
- **Security**: Built-in security features for deployment in sensitive environments
- **Scalability**: Ability to handle large-scale robotic systems with many components
- **Multi-platform support**: Compatibility across different operating systems and architectures
- **Industry readiness**: Features necessary for commercial deployment

### Architecture Foundation

ROS2 is built on DDS (Data Distribution Service), a standardized middleware protocol that handles message passing between different parts of a robotic system. This foundation provides:

- **Decentralized communication**: No single point of failure
- **Quality of Service (QoS) settings**: Configurable reliability and performance parameters
- **Language independence**: Support for multiple programming languages
- **Network transparency**: Communication works seamlessly across network boundaries

## 1.2.2 Core ROS2 Concepts

### Nodes

A **node** is a process that performs computation in a ROS2 system. Think of nodes as individual programs that work together to accomplish a complex task. In a mobile robot, you might have:

- A node controlling the motors
- A node processing sensor data
- A node performing navigation
- A node handling user interface

Nodes are the fundamental building blocks of ROS2 applications. Each node typically performs a specific function and communicates with other nodes to achieve the overall robot behavior.

### Topics

**Topics** enable asynchronous communication between nodes using a publish-subscribe model. A node publishes data to a topic, and other nodes subscribe to that topic to receive the data. This is ideal for sensor data streams like camera images, laser scans, or IMU readings.

For example, a camera node publishes images to the `/camera/image_raw` topic, while multiple other nodes (image processing, object detection, logging) can subscribe to this topic to receive the images simultaneously.

### Services

**Services** provide synchronous request-response communication between nodes. Unlike topics which are continuous, services are on-demand. When a node needs specific information or wants to trigger a specific action, it sends a request to a service and waits for a response.

For instance, a navigation node might request a path calculation from a path planning service, sending the start and goal positions and receiving the calculated path in response.

### Parameters

**Parameters** are configuration values that can be set at runtime and shared between nodes. They're used for values that might need adjustment without restarting nodes, such as sensor calibration values, control gains, or operational thresholds.

### Actions

**Actions** are goal-oriented communication that can take a long time to complete and provides feedback during execution. They're ideal for tasks like navigation to a goal position, where you want to monitor progress and have the ability to cancel the action.

## 1.2.3 ROS2 Ecosystem Components

### ROS2 Client Libraries

ROS2 supports multiple programming languages through client libraries:

- **rclcpp**: C++ client library
- **rclpy**: Python client library
- **rclrs**: Rust client library (community maintained)
- **rclc**: C client library

These libraries provide the same functionality across languages, allowing developers to use their preferred language while maintaining compatibility.

### Tools and Utilities

ROS2 comes with a rich set of command-line tools:

- `ros2 run`: Execute a specific node
- `ros2 topic`: Inspect and interact with topics
- `ros2 service`: Work with services
- `ros2 param`: Manage parameters
- `ros2 action`: Interact with actions
- `ros2 launch`: Start multiple nodes with configuration
- `rviz2`: 3D visualization tool for robot data

### Package Management

ROS2 uses packages to organize code, data, and configuration. A package typically contains:

- Source code
- Configuration files
- Launch files
- Documentation
- Tests
- Dependencies

Packages are managed using `colcon`, the ROS2 build system, which can compile packages in parallel for improved build times.

## 1.2.4 ROS2 in Physical AI Context

### Advantages for Physical AI

ROS2 provides several advantages for Physical AI systems:

- **Modularity**: Complex behaviors can be decomposed into manageable, reusable components
- **Interoperability**: Different algorithms and implementations can work together seamlessly
- **Real-time capabilities**: Support for time-critical applications essential for physical systems
- **Distributed computing**: Components can run on different computers connected over a network
- **Hardware abstraction**: Same algorithms can work with different hardware through standardized interfaces

### Communication Patterns for Physical AI

The publish-subscribe model of topics is particularly well-suited for sensor data distribution, where multiple algorithms might need the same sensor information simultaneously. Services work well for configuration and control requests that require immediate responses.

For example, a humanoid robot might have:
- Joint state information published to `/joint_states` for controllers and simulators
- Camera images published to `/camera/color/image_raw` for perception systems
- Twist commands sent via service call to `/cmd_vel` for base movement
- Parameter server holding calibration values for sensors

## 1.2.5 Quality of Service (QoS) Settings

One of ROS2's key improvements over ROS1 is its Quality of Service (QoS) system. QoS settings allow you to specify the behavior of communication between nodes:

- **Reliability**: Reliable (ensure delivery) or Best Effort (faster, occasional packet loss acceptable)
- **Durability**: Transient Local (late-joining subscribers receive last message) or Volatile (only new messages)
- **History**: Keep Last (maintain fixed number of messages) or Keep All (maintain all messages)
- **Deadline**: Maximum time between consecutive messages
- **Liveliness**: How to detect if a publisher is still active

These settings are crucial for Physical AI systems where different data types have different requirements. Sensor data might use reliable delivery with low history, while configuration data might use transient local durability.

## Key Terms

- **Node**: A process that performs computation in a ROS2 system
- **Topic**: Communication channel for asynchronous publish-subscribe messaging
- **Service**: Synchronous request-response communication mechanism
- **Parameter**: Configuration value that can be set at runtime
- **Action**: Goal-oriented communication for long-running tasks
- **DDS**: Data Distribution Service, the middleware underlying ROS2
- **QoS**: Quality of Service, configurable communication behavior settings
- **Package**: Organizational unit containing code, data, and configuration
- **Launch file**: Configuration file to start multiple nodes with settings

## Summary

ROS2 provides a robust framework for developing complex robotic systems. Its architecture based on DDS offers improved real-time performance, security, and scalability compared to ROS1. The core concepts of nodes, topics, services, parameters, and actions provide flexible communication patterns suitable for various robotics applications. For Physical AI systems, ROS2's modularity, interoperability, and distributed computing capabilities make it an ideal choice for building sophisticated robotic applications.

## Exercises

1. **Conceptual Understanding**: Explain the difference between ROS2 topics and services. When would you use each for different aspects of a Physical AI system?

2. **System Design**: Design a simple robotic system (e.g., mobile robot with camera and wheels) using ROS2 concepts. Identify what would be nodes, topics, services, and parameters in your system.

3. **QoS Analysis**: For the robotic system you designed in exercise 2, determine appropriate QoS settings for different communication channels and justify your choices.

## Next Steps

Continue to [1.3 ROS2 Architecture & Communication Patterns](./ros2-architecture-communication-patterns.md) to explore advanced ROS2 features and implementation details for Physical AI applications.
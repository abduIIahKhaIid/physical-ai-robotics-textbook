---
title: "Week 2 Quiz: ROS2 Introduction and Core Concepts"
description: "Test your understanding of ROS2 concepts and implementation for Physical AI systems."
tags: [ros2, quiz, robotics, implementation, physical-ai]
learning-objectives:
  - "Demonstrate understanding of ROS2 architecture and components"
  - "Apply ROS2 communication patterns appropriately"
  - "Configure Quality of Service settings for different scenarios"
  - "Implement ROS2 systems effectively"
---

# Week 2 Quiz: ROS2 Introduction and Core Concepts

## Learning Objectives

After completing this quiz, you will demonstrate:
- Understanding of ROS2 architecture and components
- Ability to apply ROS2 communication patterns appropriately
- Knowledge of Quality of Service configuration for different scenarios
- Proficiency in ROS2 system implementation concepts

## Quiz Instructions

- This quiz contains 10 multiple-choice questions and 2 short-answer questions
- You have unlimited attempts to take this quiz
- Aim for a score of 80% or higher before proceeding to Week 3
- Each question is worth 1 point
- Short-answer questions are worth 5 points each

## Multiple Choice Questions

### Question 1
What is the primary difference between ROS2 and ROS1?

A) ROS2 uses XML configuration while ROS1 uses YAML
B) ROS2 is built on DDS (Data Distribution Service) providing better real-time and security features
C) ROS2 only supports Python while ROS1 supports multiple languages
D) ROS2 has a centralized master node while ROS1 is decentralized

### Question 2
Which of the following is NOT a core communication concept in ROS2?

A) Nodes
B) Topics
C) Master
D) Services

### Question 3
What is the purpose of Quality of Service (QoS) settings in ROS2?

A) To limit the amount of memory used by nodes
B) To configure communication behavior including reliability and durability
C) To restrict the number of nodes that can run simultaneously
D) To encrypt messages between nodes

### Question 4
Which QoS setting would be most appropriate for high-frequency sensor data like camera images?

A) Reliable + Transient Local
B) Best Effort + Volatile
C) Reliable + Persistent
D) Best Effort + Transient Local

### Question 5
What is the main purpose of ROS2 actions?

A) To provide synchronous request-response communication
B) To handle long-running tasks with feedback and cancellation
C) To store persistent configuration parameters
D) To broadcast messages to all nodes in the system

### Question 6
Which ROS2 client library is used for Python development?

A) rclcpp
B) rclpy
C) rcljava
D) rcljs

### Question 7
What is the typical use case for ROS2 services?

A) Continuous data streaming
B) On-demand request-response communication
C) Broadcasting system-wide notifications
D) Storing configuration parameters

### Question 8
Which of the following is a valid reason for using ROS2 composition?

A) To reduce network bandwidth usage
B) To improve performance by running multiple nodes in a single process
C) To encrypt node communications
D) To limit the number of available topics

### Question 9
What does the "durability" QoS policy determine?

A) How messages are serialized
B) How long messages persist after a publisher stops
C) The maximum size of messages
D) The rate at which messages are sent

### Question 10
Which ROS2 concept is used for system-wide configuration values?

A) Topics
B) Services
C) Parameters
D) Actions

## Short Answer Questions

### Question 11 (5 points)
Explain the difference between ROS2 topics and services. Provide specific examples of when you would use each for different aspects of a Physical AI system, considering the real-time requirements and reliability needs.

### Question 12 (5 points)
Describe how Quality of Service (QoS) settings would differ for three types of communication in a mobile robot system: (1) camera image streaming, (2) emergency stop commands, and (3) robot configuration parameters. Justify your choices for each scenario.

## Answer Key

### Multiple Choice Answers:
1. B) ROS2 is built on DDS (Data Distribution Service) providing better real-time and security features
2. C) Master
3. B) To configure communication behavior including reliability and durability
4. B) Best Effort + Volatile
5. B) To handle long-running tasks with feedback and cancellation
6. B) rclpy
7. B) On-demand request-response communication
8. B) To improve performance by running multiple nodes in a single process
9. B) How long messages persist after a publisher stops
10. C) Parameters

### Sample Short Answer Responses:

**Question 11:**
ROS2 topics provide asynchronous, publish-subscribe communication ideal for continuous data streams like sensor readings. They allow multiple subscribers to receive the same data simultaneously. Services provide synchronous request-response communication suitable for on-demand operations that require immediate responses. For Physical AI systems: Topics are used for sensor data distribution (e.g., camera images, laser scans) where multiple algorithms need the same data continuously. Services are used for configuration changes or computations that must complete before continuing (e.g., requesting a path calculation, changing operational mode).

**Question 12:**
For camera image streaming: Best Effort + Volatile + Keep Last (depth=1) because occasional dropped frames are acceptable but speed is crucial. For emergency stop commands: Reliable + Volatile + Keep Last (depth=1) because delivery must be guaranteed but only the most recent command matters. For robot configuration parameters: Reliable + Transient Local + Keep Last (depth=1) because new subscribers need the current configuration, delivery must be guaranteed, and only the latest value matters.

## Summary

This quiz assessed your understanding of ROS2 core concepts including architecture, communication patterns, Quality of Service settings, and implementation strategies. These concepts are fundamental to building robust Physical AI systems that can handle the real-time demands and reliability requirements of physical interaction.

## Next Steps

Achieve a score of 80% or higher before proceeding to [1.3 Embodied Cognition & Environmental Interaction](../1.3-embodied-cognition-and-environmental-interaction/index.md). If you scored below 80%, review the 1.2 materials before retaking the quiz.
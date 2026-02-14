---
title: "Week 2 Lab: ROS2 Node Implementation"
description: "Practical exercises for ROS2 node creation."
tags: [ros2, exercises, lab]
learning-objectives:
  - "Implement ROS2 publisher and subscriber nodes"
---

# Week 2 Lab: ROS2 Node Implementation

## Exercise 1: Basic Publisher

Create a publisher node that sends counter messages.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class CounterPublisher(Node):
    def __init__(self):
        super().__init__('counter_publisher')
        self.publisher = self.create_publisher(Int32, 'counter', 10)
        self.timer = self.create_timer(1.0, self.publish_count)
        self.count = 0

    def publish_count(self):
        msg = Int32()
        msg.data = self.count
        self.publisher.publish(msg)
        self.count += 1
```

## Exercise 2: Basic Subscriber

Create a subscriber that receives and logs messages.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class CounterSubscriber(Node):
    def __init__(self):
        super().__init__('counter_subscriber')
        self.subscription = self.create_subscription(
            Int32, 'counter', self.listener_callback, 10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
```

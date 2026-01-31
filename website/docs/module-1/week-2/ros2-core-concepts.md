---
title: "1.3 ROS2 Core Concepts and Implementation"
description: "Deep dive into ROS2 core concepts with practical implementation details for Physical AI systems."
tags: [ros2, core-concepts, implementation, robotics, physical-ai]
learning-objectives:
  - "Implement ROS2 nodes in both C++ and Python"
  - "Create and use custom message types"
  - "Design effective topic and service architectures"
  - "Apply Quality of Service settings appropriately"
---

# 1.3 ROS2 Core Concepts and Implementation

## Learning Objectives

By the end of this chapter, you will be able to:
- Implement ROS2 nodes in both C++ and Python
- Create and use custom message types
- Design effective topic and service architectures for Physical AI systems
- Apply Quality of Service settings appropriately for different use cases
- Understand and implement ROS2 launch systems
- Configure and manage ROS2 parameters effectively

## Introduction

Now that we understand the theoretical foundation of ROS2, let's dive into practical implementation. This chapter will guide you through creating actual ROS2 components, from basic nodes to complex message types and communication patterns. Understanding these implementation details is crucial for building robust Physical AI systems that can handle the real-time demands and reliability requirements of physical interaction.

## 1.3.1 Creating ROS2 Nodes

### Node Structure

A ROS2 node consists of several key components:

1. **Initialization**: Setting up the node and communication infrastructure
2. **Publishers/Subscribers**: Establishing communication channels
3. **Timers**: For periodic execution of tasks
4. **Callbacks**: Functions that handle incoming messages
5. **Cleanup**: Proper shutdown procedures

### Python Implementation Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### C++ Implementation Example

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello World: " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclpy::init(argc, argv);
  rclpy::spin(std::make_shared<MinimalPublisher>());
  rclpy::shutdown();
  return 0;
}
```

### Node Lifecycle Management

ROS2 nodes can participate in a lifecycle system that provides better control over startup, shutdown, and recovery. This is particularly important for Physical AI systems where graceful startup and shutdown can prevent hardware damage.

The lifecycle states include:
- Unconfigured
- Inactive
- Active
- Finalized

## 1.3.2 Message Types and Custom Messages

### Standard Message Types

ROS2 provides a rich set of standard message types in the `std_msgs` package:

- `Bool`, `Int32`, `Float64`: Basic data types
- `String`: Text data
- `Header`: Timestamp and frame information
- `ColorRGBA`: Color information
- `Empty`: No data, for signaling

For robotics-specific data, there are packages like:
- `geometry_msgs`: Pose, twist, point, vector3d
- `sensor_msgs`: Image, LaserScan, JointState
- `nav_msgs`: OccupancyGrid, Path, Odometry

### Creating Custom Messages

Custom messages are defined in `.msg` files and stored in the `msg/` directory of a package:

```
# Point2D.msg
float64 x
float64 y
---
# Pose2D.msg
Point2D position
float64 theta
```

After defining custom messages, they must be registered in the package's CMakeLists.txt and setup.py files.

### Message Serialization

Messages are serialized using the ROS Interface Definition Language (IDL) with support for multiple serialization formats:
- **ROS**: Default ROS2 serialization
- **JSON**: Human-readable format
- **CycloneDDS**: Efficient binary format

## 1.3.3 Quality of Service (QoS) in Practice

### QoS Profiles

ROS2 provides predefined QoS profiles for common use cases:

- `sensor_data_qos`: Best effort, volatile durability for sensor data
- `services_default_qos`: Reliable communication for services
- `parameters_qos`: Transient local for parameter server
- `system_default_qos`: Default settings for general use

### Practical QoS Selection

For Physical AI systems, QoS selection is critical:

**High-frequency sensor data** (e.g., camera images, IMU):
```python
qos_profile = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE
)
```

**Critical control commands** (e.g., motor commands):
```python
qos_profile = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)
```

**Configuration data** (e.g., calibration parameters):
```python
qos_profile = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)
```

## 1.3.4 Services and Actions Implementation

### Services

Services provide synchronous request-response communication:

```python
# Service server
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a}, b: {request.b}')
        return response
```

### Actions

Actions are designed for long-running tasks with feedback:

```python
# Action interface (Fibonacci.action)
goal:
  int32 order
result:
  int32[] sequence
feedback:
  int32[] sequence
```

Action servers provide feedback during execution and can be preempted:

```python
class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Fibonacci.Result()

            feedback_msg.sequence.append(feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result
```

## 1.3.5 Launch Files and System Composition

### Launch Files

Launch files allow you to start multiple nodes with specific configurations:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker',
            parameters=[
                {'param_name': 'param_value'}
            ],
            remappings=[
                ('original_topic', 'remapped_topic')
            ]
        ),
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='listener'
        )
    ])
```

### Composition

For performance-critical applications, multiple nodes can be composed into a single process:

```cpp
#include "composition/composition.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  composition::add_node_to_executor("composition", "Talker", exec);
  composition::add_node_to_executor("composition", "Listener", exec);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
```

## 1.3.6 Parameter Management

### Parameter Declaration and Usage

Parameters can be declared with default values and constraints:

```python
def __init__(self):
    super().__init__('parameter_node')
    self.declare_parameter('my_param', 'default_value')
    self.declare_parameter('threshold', 0.5, ParameterDescriptor(
        name='threshold',
        type=ParameterType.PARAMETER_DOUBLE,
        description='Threshold value for detection',
        floating_point_range=[ParameterRange(from_value=0.0, to_value=1.0, step=0.01)]
    ))
```

### Dynamic Parameters

ROS2 supports dynamic parameter reconfiguration:

```python
from rcl_interfaces.msg import SetParametersResult

def parameter_callback(self, params):
    for param in params:
        if param.name == 'threshold' and param.type_ == Parameter.Type.DOUBLE:
            if param.value < 0.0 or param.value > 1.0:
                return SetParametersResult(successful=False, reason='Threshold out of range')
    return SetParametersResult(successful=True)
```

## 1.3.7 ROS2 for Physical AI Systems

### Real-time Considerations

For Physical AI systems, real-time performance is crucial:

- Use real-time capable DDS implementations (e.g., RTI Connext)
- Configure thread priorities appropriately
- Minimize communication overhead
- Use efficient message serialization

### Safety and Reliability

Physical AI systems require enhanced safety measures:

- Implement heartbeat mechanisms
- Use appropriate QoS for safety-critical communications
- Implement fault detection and recovery
- Log all safety-relevant events

### Hardware Integration

ROS2 bridges the gap between high-level algorithms and hardware:

- Hardware Abstraction Layer (HAL) nodes
- Device drivers following ROS2 conventions
- Sensor fusion and calibration nodes
- Safety monitoring nodes

## Key Terms

- **Node Lifecycle**: Managed states of a ROS2 node (unconfigured, inactive, active, finalized)
- **Custom Messages**: User-defined message types for specific data structures
- **QoS Profiles**: Predefined Quality of Service configurations for common use cases
- **Action Server**: Component that handles long-running tasks with feedback
- **Composition**: Running multiple nodes in a single process for performance
- **Parameter Descriptors**: Metadata that describes parameter properties and constraints
- **Dynamic Parameters**: Parameters that can be changed at runtime with validation

## Summary

This chapter covered the practical implementation aspects of ROS2, from basic node creation to advanced features like actions and composition. Understanding these concepts is essential for building robust Physical AI systems that can handle the real-time demands and reliability requirements of physical interaction. The combination of proper QoS configuration, appropriate message types, and effective system composition forms the foundation for complex robotic applications.

## Exercises

1. **Node Implementation**: Create a ROS2 node that subscribes to sensor data (e.g., laser scan) and publishes processed information (e.g., obstacle distances). Implement both Python and C++ versions.

2. **Custom Messages**: Design and implement custom message types for a specific Physical AI application (e.g., humanoid robot joint commands with safety limits).

3. **QoS Configuration**: For the system you designed in exercise 1, determine and implement appropriate QoS settings for each communication channel, justifying your choices.

4. **Service Implementation**: Create a ROS2 service that performs a computation (e.g., inverse kinematics) and implement both the server and client sides.

5. **Launch File**: Create a launch file that starts multiple nodes with appropriate parameters and remappings for a complete Physical AI subsystem.

## Next Steps

Continue to [Week 2 Exercises](./exercises.md) to apply these concepts through practical hands-on exercises.
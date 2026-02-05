---
title: Forward Kinematics in Humanoid Systems
sidebar_position: 1
description: Advanced mathematical foundations for determining end-effector positions from joint configurations in humanoid robotics
---

# Forward Kinematics

Forward kinematics is a fundamental concept in robotics that describes the process of calculating the position and orientation of a robot's end-effector based on the known joint angles. In humanoid robotics, this is essential for understanding where limbs are positioned in space given their joint configurations.

## Understanding Forward Kinematics

Forward kinematics answers the question: "Where is the end of the robot's arm/leg given the current joint angles?"

In mathematical terms, forward kinematics transforms joint space coordinates into Cartesian space coordinates using transformation matrices derived from the Denavit-Hartenberg (DH) convention or other kinematic representations.

## Key Components

### Joint Space vs. Cartesian Space
- **Joint Space**: Describes robot configuration in terms of joint angles (θ₁, θ₂, θ₃, ...)
- **Cartesian Space**: Describes end-effector position in 3D space (x, y, z) and orientation (roll, pitch, yaw)

### Transformation Matrices
Transformation matrices combine rotation and translation operations to map points from one coordinate frame to another. For each joint, we calculate a transformation matrix that describes its pose relative to the previous joint.

## Humanoid Robot Application

In humanoid robots, forward kinematics is used for:
- Arm positioning for reaching tasks
- Leg positioning for walking and balance
- Head orientation for gaze control
- Whole-body posture calculation

## Mathematical Representation

For a robotic arm with n joints, the forward kinematics equation is:

**T₀ⁿ = T₀¹(θ₁) × T₁²(θ₂) × ... × T<sub>n-1</sub>ⁿ(θ<sub>n</sub>)**

Where:
- T₀ⁿ is the final transformation matrix from base to end-effector
- Tᵢ<sup>i+1</sup>(θᵢ₊₁) represents the transformation from joint i to joint i+1

## Example: Simple 2-DOF Arm

Consider a simple planar arm with 2 rotational joints:
- Joint 1: Rotates around z-axis at the base
- Joint 2: Rotates around z-axis at the elbow
- Link lengths: L₁, L₂

The end-effector position (x, y) is calculated as:
- x = L₁cos(θ₁) + L₂cos(θ₁ + θ₂)
- y = L₁sin(θ₁) + L₂sin(θ₁ + θ₂)

## Implementation in Code

```python
import numpy as np

def forward_kinematics_2dof(theta1, theta2, l1, l2):
    """
    Calculate forward kinematics for a 2-DOF planar arm

    Args:
        theta1: Angle of first joint (radians)
        theta2: Angle of second joint (radians)
        l1: Length of first link
        l2: Length of second link

    Returns:
        tuple: (x, y) end-effector position
    """
    x = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2)
    y = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2)

    return x, y

# Example usage
theta1 = np.pi/4  # 45 degrees
theta2 = np.pi/6  # 30 degrees
l1 = 1.0
l2 = 0.8

position = forward_kinematics_2dof(theta1, theta2, l1, l2)
print(f"End-effector position: ({position[0]:.2f}, {position[1]:.2f})")
```

## Humanoid-Specific Considerations

### Multiple Chains
Humanoid robots typically have multiple kinematic chains (arms, legs) that must be calculated simultaneously.

### Joint Limits
Real joints have physical limits that constrain the reachable workspace.

### Redundancy
Humanoid arms and legs often have more degrees of freedom than required for basic positioning, leading to multiple solutions for the same end-effector pose.

## Exercise

Calculate the position of a humanoid robot's hand given the following simplified arm configuration:
- Shoulder joint angle (θ₁): π/3 radians (60°)
- Elbow joint angle (θ₂): π/4 radians (45°)
- Shoulder-to-elbow length: 0.3 m
- Elbow-to-hand length: 0.25 m

What is the (x, y) position of the hand if the shoulder is located at the origin (0, 0)?

**Solution**: Apply the forward kinematics equations for a 2-DOF arm to determine the hand position.

## Summary

Forward kinematics provides the mathematical foundation for understanding how humanoid robot joints relate to end-effector positions. Mastering this concept is essential for controlling robot movements and predicting their behavior in various tasks.
---
title: Inverse Kinematics for Humanoid Robots
sidebar_position: 2
description: Advanced mathematical techniques for determining joint configurations to achieve desired end-effector poses in humanoid systems
---

# Inverse Kinematics

Inverse kinematics (IK) is the reverse process of forward kinematics, where we determine the joint angles required to position the robot's end-effector at a desired location and orientation. This is crucial for humanoid robots to reach targets, maintain balance, and perform goal-oriented tasks.

## Understanding Inverse Kinematics

While forward kinematics asks "Where is the end-effector given these joint angles?", inverse kinematics asks "What joint angles are needed to place the end-effector here?"

Mathematically, IK solves for joint angles θ given a desired end-effector pose T<sub>desired</sub>.

## Key Challenges

### Analytical vs. Numerical Solutions
- **Analytical Solutions**: Closed-form mathematical expressions that directly solve for joint angles (possible for simple kinematic chains)
- **Numerical Solutions**: Iterative methods that approximate solutions (necessary for complex humanoid structures)

### Multiple Solutions
Many kinematic chains have multiple valid solutions for the same end-effector pose, especially redundant systems like humanoid arms.

### Singularity Handling
At singular configurations, small changes in end-effector position may require large or impossible joint movements.

## Common Solution Methods

### Jacobian-Based Methods
The Jacobian matrix relates joint velocities to end-effector velocities:

**J(θ) = ∂f(θ)/∂θ**

For small changes, we can approximate:
**Δθ = J⁻¹(θ) × Δx**

### Cyclic Coordinate Descent (CCD)
An iterative method that adjusts each joint angle sequentially to move the end-effector closer to the target.

### Pseudoinverse Method
Using the pseudoinverse of the Jacobian to handle redundant systems:
**Δθ = J⁺ × Δx**, where J⁺ = Jᵀ(JJᵀ + λI)⁻¹

## Humanoid Robot Applications

### Reaching Motions
Determining arm configurations to reach specific points in space.

### Walking and Balance
Calculating leg poses to maintain center of mass and achieve stable gaits.

### Whole-Body Control
Coordinating multiple kinematic chains simultaneously for complex movements.

## Implementation Example

```python
import numpy as np

def jacobian_2dof(theta1, theta2, l1, l2):
    """
    Calculate the Jacobian matrix for a 2-DOF planar arm

    Args:
        theta1: Angle of first joint (radians)
        theta2: Angle of second joint (radians)
        l1: Length of first link
        l2: Length of second link

    Returns:
        numpy.ndarray: 2x2 Jacobian matrix
    """
    # Partial derivatives of position equations
    J = np.array([
        [-l1*np.sin(theta1) - l2*np.sin(theta1 + theta2), -l2*np.sin(theta1 + theta2)],
        [l1*np.cos(theta1) + l2*np.cos(theta1 + theta2), l2*np.cos(theta1 + theta2)]
    ])

    return J

def inverse_kinematics_jacobian(start_theta1, start_theta2, target_x, target_y,
                               l1, l2, max_iterations=100, tolerance=1e-6):
    """
    Solve inverse kinematics using Jacobian-based method

    Args:
        start_theta1, start_theta2: Initial joint angles
        target_x, target_y: Desired end-effector position
        l1, l2: Link lengths
        max_iterations: Maximum number of iterations
        tolerance: Position tolerance for convergence

    Returns:
        tuple: (theta1, theta2) if successful, None if failed
    """
    theta = np.array([start_theta1, start_theta2])

    for i in range(max_iterations):
        # Calculate current position
        current_x = l1 * np.cos(theta[0]) + l2 * np.cos(theta[0] + theta[1])
        current_y = l1 * np.sin(theta[0]) + l2 * np.sin(theta[0] + theta[1])

        # Calculate error
        error = np.array([target_x - current_x, target_y - current_y])

        # Check for convergence
        if np.linalg.norm(error) < tolerance:
            return theta[0], theta[1]

        # Calculate Jacobian and update
        J = jacobian_2dof(theta[0], theta[1], l1, l2)
        try:
            delta_theta = np.linalg.inv(J) @ error
            theta += delta_theta * 0.1  # Small step to ensure stability
        except np.linalg.LinAlgError:
            # Handle singularity
            print("Singularity encountered, adjusting step size")
            delta_theta = np.linalg.pinv(J) @ error
            theta += delta_theta * 0.01
            if i > max_iterations * 0.9:
                print("Near singularity, using pseudoinverse")

    print("Max iterations reached without convergence")
    return None

# Example usage
target_pos = (1.0, 0.5)  # Desired end-effector position
l1, l2 = 0.8, 0.6  # Link lengths

result = inverse_kinematics_jacobian(
    start_theta1=np.pi/4,
    start_theta2=np.pi/6,
    target_x=target_pos[0],
    target_y=target_pos[1],
    l1=l1,
    l2=l2
)

if result:
    theta1, theta2 = result
    print(f"Required joint angles: θ1={theta1:.3f} rad, θ2={theta2:.3f} rad")
else:
    print("Could not reach target position")
```

## Humanoid-Specific Considerations

### Redundancy Resolution
Humanoid robots have more degrees of freedom than required for basic positioning. Additional constraints (like minimizing energy or staying within joint limits) help select optimal solutions.

### Constraints
- Joint angle limits
- Collision avoidance
- Balance maintenance
- Actuator force/torque limits

### Real-time Performance
Humanoid robots often need real-time IK solutions for walking and manipulation, requiring efficient algorithms and computational optimization.

## Exercise

A humanoid robot needs to reach a point (0.7, 0.5) meters relative to its shoulder with an arm consisting of:
- Upper arm length: 0.4 m
- Forearm length: 0.35 m
- Starting joint angles: θ₁ = π/4, θ₂ = π/6

Use the Jacobian-based inverse kinematics method to find the required joint angles to reach this position.

**Hint**: Implement the algorithm and experiment with different starting positions and step sizes to understand convergence behavior.

## Summary

Inverse kinematics enables goal-directed movements in humanoid robots by calculating the necessary joint configurations. While analytical solutions exist for simple cases, numerical methods are essential for the complex, redundant structures found in humanoid robots.
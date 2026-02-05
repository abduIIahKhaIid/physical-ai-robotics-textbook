---
title: Kinematic Analysis and Control Laboratory
sidebar_position: 4
description: Comprehensive laboratory exercises for implementing and analyzing kinematic systems in humanoid robotics
---

# Kinematics Lab

This lab provides hands-on exercises to reinforce your understanding of forward and inverse kinematics concepts applied to humanoid robots. Through these exercises, you'll practice calculating robot positions, solving for joint angles, and understanding coordinate system transformations.

## Lab Objectives

After completing this lab, you will be able to:
- Calculate forward kinematics for simple and complex robot configurations
- Implement inverse kinematics solutions using different methods
- Apply coordinate system transformations to real-world scenarios
- Understand the practical challenges in kinematic calculations

## Exercise 1: Simple Planar Arm

### Setup
Consider a planar robot arm with 2 joints in the X-Y plane:
- Joint 1: Rotates around Z-axis at origin (0, 0, 0)
- Joint 2: Rotates around Z-axis at elbow
- Link 1 length (shoulder to elbow): 0.3 meters
- Link 2 length (elbow to hand): 0.25 meters

### Tasks
1. **Forward Kinematics**: Calculate the hand position when:
   - θ₁ = π/3 (60°), θ₂ = π/6 (30°)
   - θ₁ = π/4 (45°), θ₂ = π/4 (45°)

2. **Inverse Kinematics**: Find joint angles to reach target positions:
   - Target: (0.4, 0.3)
   - Target: (0.5, 0.1)

3. **Workspace Analysis**: Plot the reachable workspace for this arm

### Verification
Check that your solutions satisfy the forward kinematics equations:
- x = L₁cos(θ₁) + L₂cos(θ₁ + θ₂)
- y = L₁sin(θ₁) + L₂sin(θ₁ + θ₂)

## Exercise 2: 3-DOF Humanoid Arm Model

### Setup
Extend to a 3-DOF arm model (like a simplified human arm):
- Shoulder joint: rotation around Z-axis
- Elbow joint: rotation around Y-axis
- Wrist joint: rotation around Z-axis (wrist pronation/supination)

Link lengths:
- Shoulder to elbow: 0.3 m
- Elbow to wrist: 0.25 m
- Wrist to finger tip: 0.08 m

### Tasks
1. **Forward Kinematics**: Calculate end-effector position and orientation given joint angles (θ₁, θ₂, θ₃)

2. **Inverse Kinematics**: Find a solution for reaching (0.4, 0.2, 1.0) from a shoulder located at (0, 0.15, 1.2)

3. **Redundancy Exploration**: Find multiple solutions for the same target position

### Implementation
```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class HumanoidArm3DOF:
    def __init__(self, shoulder_pos=(0, 0.15, 1.2)):
        self.shoulder_pos = np.array(shoulder_pos)
        self.l1 = 0.3  # shoulder to elbow
        self.l2 = 0.25 # elbow to wrist
        self.l3 = 0.08 # wrist to fingertip

    def forward_kinematics(self, theta1, theta2, theta3):
        """
        Calculate 3D position of end-effector given joint angles

        Args:
            theta1: Shoulder rotation around Z-axis
            theta2: Elbow rotation around Y-axis
            theta3: Wrist rotation around Z-axis

        Returns:
            tuple: (x, y, z) end-effector position
        """
        # Simplified calculation for demonstration
        # In reality, you'd use full 3D transformations

        # Calculate elbow position
        elbow_x = self.shoulder_pos[0] + self.l1 * np.cos(theta1) * np.cos(theta2)
        elbow_y = self.shoulder_pos[1] + self.l1 * np.sin(theta1) * np.cos(theta2)
        elbow_z = self.shoulder_pos[2] + self.l1 * np.sin(theta2)

        # Calculate wrist/end-effector position
        end_x = elbow_x + self.l2 * np.cos(theta1) * np.cos(theta2 + np.pi/2)
        end_y = elbow_y + self.l2 * np.sin(theta1) * np.cos(theta2 + np.pi/2)
        end_z = elbow_z + self.l2 * np.sin(theta2 + np.pi/2)

        return (end_x, end_y, end_z)

    def plot_arm(self, theta1, theta2, theta3):
        """Visualize the arm configuration"""
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Calculate joint positions
        shoulder = self.shoulder_pos
        elbow_x = shoulder[0] + self.l1 * np.cos(theta1) * np.cos(theta2)
        elbow_y = shoulder[1] + self.l1 * np.sin(theta1) * np.cos(theta2)
        elbow_z = shoulder[2] + self.l1 * np.sin(theta2)
        elbow = np.array([elbow_x, elbow_y, elbow_z])

        end_effector = self.forward_kinematics(theta1, theta2, theta3)

        # Plot arm links
        joints = np.array([shoulder, elbow, end_effector])
        ax.plot(joints[:, 0], joints[:, 1], joints[:, 2], 'bo-', linewidth=3, markersize=8)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3-DOF Humanoid Arm')

        plt.show()

# Example usage
arm = HumanoidArm3DOF()

# Test forward kinematics
angles = (np.pi/4, np.pi/6, np.pi/8)  # 45°, 30°, 22.5°
pos = arm.forward_kinematics(*angles)
print(f"End-effector position: {pos}")

# Visualize arm
# arm.plot_arm(*angles)
```

## Exercise 3: Walking Pattern Calculation

### Setup
Model the leg of a humanoid robot for walking. Consider a 3-DOF leg (hip, knee, ankle) in the sagittal plane (forward-back movement).

### Tasks
1. **Step Generation**: Calculate the leg configuration needed to lift the foot 0.1m and step forward 0.3m

2. **Balance Maintenance**: Calculate how hip joint angles need to compensate for the shifted center of mass during a step

3. **Gait Cycle**: Design a simple gait cycle showing joint angle trajectories over time

### Implementation
```python
class HumanoidLeg:
    def __init__(self, hip_height=0.8, thigh_length=0.4, shin_length=0.4):
        self.hip_height = hip_height
        self.thigh = thigh_length
        self.shin = shin_length

    def step_trajectory(self, step_length, step_height, num_points=20):
        """
        Generate a trajectory for a single step

        Args:
            step_length: Distance to step forward
            step_height: Maximum height to lift foot
            num_points: Number of trajectory points

        Returns:
            list: Trajectory points [(x, y, z), ...]
        """
        trajectory = []

        # Generate step trajectory using a half-ellipse
        for i in range(num_points):
            t = i / (num_points - 1)  # Normalized time parameter (0 to 1)

            # Forward progression
            x = t * step_length

            # Height profile (elliptical arc)
            h = step_height * np.sin(t * np.pi)

            # Add to starting position (foot initially at origin)
            trajectory.append((x, 0, h))

        return trajectory

# Test step trajectory generation
leg = HumanoidLeg()
step_traj = leg.step_trajectory(step_length=0.3, step_height=0.1)
print(f"Step trajectory points: {len(step_traj)}")
print(f"First few points: {step_traj[:5]}")

# Plot trajectory
import matplotlib.pyplot as plt

traj = np.array(step_traj)
plt.figure(figsize=(10, 4))
plt.subplot(1, 2, 1)
plt.plot(traj[:, 0], traj[:, 2])
plt.title('Foot Trajectory: X vs Height')
plt.xlabel('X Position (m)')
plt.ylabel('Height (m)')

plt.subplot(1, 2, 2)
plt.plot(range(len(traj)), traj[:, 2])
plt.title('Foot Height Over Time')
plt.xlabel('Time Step')
plt.ylabel('Height (m)')

plt.tight_layout()
plt.show()
```

## Exercise 4: Coordination Challenge

### Setup
A humanoid robot needs to reach for an object while maintaining balance. Both arms must coordinate their movements.

### Tasks
1. **Reaching Motion**: Plan reaching trajectories for both arms to grasp an object at (0.6, 0, 1.0)

2. **Balance Compensation**: Calculate how leg joints should adjust to counteract reaching forces

3. **Temporal Coordination**: Sequence the arm movements to maintain balance throughout the reach

### Implementation Framework
```python
class WholeBodyController:
    def __init__(self):
        self.left_arm = HumanoidArm3DOF(shoulder_pos=(-0.1, 0.15, 1.2))  # Left shoulder
        self.right_arm = HumanoidArm3DOF(shoulder_pos=(0.1, 0.15, 1.2))   # Right shoulder
        self.legs = HumanoidLeg()

    def coordinated_reach(self, target_pos, approach_dir=(0, -1, 0)):
        """
        Plan coordinated arm movement with balance compensation

        Args:
            target_pos: (x, y, z) position to reach
            approach_dir: Direction vector for approaching the target

        Returns:
            dict: Joint angles for all limbs to achieve coordinated reach
        """
        # Calculate inverse kinematics for reaching arm(s)
        # Calculate balance adjustments for legs
        # Return complete joint configuration
        pass

# Test coordinated reach
controller = WholeBodyController()
target = (0.6, 0, 1.0)
# result = controller.coordinated_reach(target)
```

## Exercise 5: Real-world Constraints

### Setup
Real humanoid robots have joint limits, actuator constraints, and safety requirements that must be considered.

### Tasks
1. **Joint Limit Enforcement**: Modify your inverse kinematics solver to respect joint limits

2. **Singularity Avoidance**: Detect and handle singular configurations where the Jacobian becomes singular

3. **Collision Detection**: Ensure calculated configurations don't result in self-collision

### Implementation
```python
def enforce_joint_limits(angles, min_limits, max_limits):
    """
    Clamp joint angles to respect mechanical limits

    Args:
        angles: Array of joint angles
        min_limits: Array of minimum allowed angles
        max_limits: Array of maximum allowed angles

    Returns:
        numpy.ndarray: Angles within limits
    """
    return np.clip(angles, min_limits, max_limits)

def detect_singularity(jacobian, threshold=1e-6):
    """
    Check if the Jacobian matrix is near a singularity

    Args:
        jacobian: 6x6 Jacobian matrix (or appropriate size)
        threshold: Singular value threshold

    Returns:
        bool: True if near singularity
    """
    s = np.linalg.svd(jacobian, compute_uv=False)
    return s[-1] < threshold  # Check smallest singular value

# Example with limits for a 3-DOF arm
min_limits = np.array([-np.pi/2, -np.pi/3, -np.pi/4])  # Conservative limits
max_limits = np.array([np.pi/2, np.pi/2, np.pi/4])

test_angles = np.array([2*np.pi/3, np.pi/4, np.pi/3])  # Exceeds some limits
safe_angles = enforce_joint_limits(test_angles, min_limits, max_limits)
print(f"Original angles: {test_angles}")
print(f"Safely limited: {safe_angles}")
```

## Verification and Assessment

### Self-Assessment Questions
1. How do you verify that your forward kinematics calculations are correct?
2. What are the advantages and disadvantages of analytical vs. numerical inverse kinematics?
3. Why is coordinate system consistency important in humanoid robotics?
4. How do joint limits affect the reachable workspace of a humanoid robot?

### Practical Tests
1. Implement the forward kinematics for your chosen robot configuration
2. Verify results by plotting the robot in different configurations
3. Test the inverse kinematics solver with known positions
4. Ensure all solutions produce valid forward kinematics results

## Extension Activities

1. **Simulation Integration**: Connect your kinematic calculations to a robot simulator like Gazebo or PyBullet
2. **Motion Planning**: Add path planning to move between positions without collisions
3. **Dynamic Modeling**: Incorporate velocity and acceleration constraints
4. **Learning Approach**: Use neural networks to approximate inverse kinematics solutions

## Summary

This lab provides hands-on experience with fundamental kinematic concepts essential for humanoid robotics. Practice these calculations thoroughly, as they form the foundation for all robot movement and interaction tasks. Remember to consider real-world constraints like joint limits, singularity, and safety when implementing your solutions.
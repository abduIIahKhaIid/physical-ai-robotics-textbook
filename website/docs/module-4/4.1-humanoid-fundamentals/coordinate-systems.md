---
title: Coordinate Systems and Spatial Transformations
sidebar_position: 3
description: Mathematical foundations for spatial reasoning and reference frame management in humanoid robotics
---

# Coordinate Systems

Coordinate systems provide the mathematical framework for describing positions, orientations, and transformations in robotic systems. In humanoid robotics, proper coordinate system management is crucial for accurate movement, interaction, and spatial reasoning.

## Types of Coordinate Systems

### World Coordinate System
- Fixed reference frame for the entire environment
- Typically defined as (x, y, z) with z pointing upward
- All other coordinate systems are referenced relative to this

### Base Coordinate System
- Fixed to the robot's base or torso
- Origin is usually at the robot's center of mass or geometric center
- Defines the robot's reference frame

### Joint Coordinate Systems
- Defined at each joint to describe local movement
- Each joint has its own coordinate system that moves with it
- Essential for calculating transformations along kinematic chains

### End-Effector Coordinate System
- Attached to the robot's end-effector (hand, foot, head)
- Moves with the end-effector relative to the base
- Often called the tool coordinate system

## Coordinate System Transformations

### Homogeneous Transformations
Homogeneous coordinates (4×4 matrices) allow us to represent both rotation and translation in a single matrix:

```
T = [R  p]
    [0  1]
```

Where R is a 3×3 rotation matrix and p is a 3×1 position vector.

### Rotation Matrices
Rotation matrices transform vectors from one coordinate frame to another:

```
R_x(α) = [1   0       0     ]
         [0   cos α  -sin α ]
         [0   sin α   cos α ]

R_y(β) = [ cos β  0   sin β ]
         [ 0      1     0   ]
         [-sin β  0   cos β ]

R_z(γ) = [cos γ  -sin γ  0]
         [sin γ   cos γ  0]
         [0       0      1]
```

## Humanoid Robot Coordinate Systems

### Body Frame Definitions
In humanoid robotics, standard conventions include:
- X-axis: Forward direction (anterior/posterior)
- Y-axis: Lateral direction (left/right)
- Z-axis: Vertical direction (superior/inferior)

### Joint Frame Conventions
- Each joint defines its own coordinate system following the Denavit-Hartenberg (DH) convention
- Consistent frame assignment ensures proper kinematic modeling

## Implementation Example

```python
import numpy as np

class CoordinateSystem:
    def __init__(self, name, transform_matrix=None):
        """
        Initialize a coordinate system with an optional transform matrix

        Args:
            name: String identifier for the coordinate system
            transform_matrix: 4x4 homogeneous transformation matrix
        """
        self.name = name
        if transform_matrix is None:
            self.transform = np.eye(4)  # Identity matrix
        else:
            self.transform = transform_matrix

    def translate(self, dx, dy, dz):
        """Add translation to the coordinate system"""
        translation = np.array([
            [1, 0, 0, dx],
            [0, 1, 0, dy],
            [0, 0, 1, dz],
            [0, 0, 0, 1]
        ])
        self.transform = self.transform @ translation
        return self

    def rotate_x(self, angle):
        """Rotate around X axis"""
        cos_a, sin_a = np.cos(angle), np.sin(angle)
        rotation = np.array([
            [1, 0, 0, 0],
            [0, cos_a, -sin_a, 0],
            [0, sin_a, cos_a, 0],
            [0, 0, 0, 1]
        ])
        self.transform = self.transform @ rotation
        return self

    def rotate_y(self, angle):
        """Rotate around Y axis"""
        cos_a, sin_a = np.cos(angle), np.sin(angle)
        rotation = np.array([
            [cos_a, 0, sin_a, 0],
            [0, 1, 0, 0],
            [-sin_a, 0, cos_a, 0],
            [0, 0, 0, 1]
        ])
        self.transform = self.transform @ rotation
        return self

    def rotate_z(self, angle):
        """Rotate around Z axis"""
        cos_a, sin_a = np.cos(angle), np.sin(angle)
        rotation = np.array([
            [cos_a, -sin_a, 0, 0],
            [sin_a, cos_a, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        self.transform = self.transform @ rotation
        return self

    def transform_point(self, point):
        """Transform a 3D point using this coordinate system"""
        homogeneous_point = np.append(point, 1)
        transformed = self.transform @ homogeneous_point
        return transformed[:3]

    def get_position(self):
        """Get the position component of the transform"""
        return self.transform[:3, 3]

    def get_orientation(self):
        """Get the rotation matrix component of the transform"""
        return self.transform[:3, :3]

def create_humanoid_arm_coordinate_systems():
    """
    Create coordinate systems for a humanoid arm with shoulder, elbow, and wrist joints

    Returns:
        dict: Dictionary containing coordinate systems for each joint
    """
    # World coordinate system (fixed environment frame)
    world = CoordinateSystem("world")

    # Torso coordinate system (attached to robot's torso)
    torso = CoordinateSystem("torso",
                           np.array([[1, 0, 0, 0],
                                    [0, 1, 0, 0],
                                    [0, 0, 1, 1.2],  # Robot torso height
                                    [0, 0, 0, 1]]))

    # Shoulder coordinate system (relative to torso)
    shoulder = CoordinateSystem("shoulder",
                              np.array([[1, 0, 0, 0.2],  # Offset from torso center
                                       [0, 1, 0, 0.15],  # Lateral offset
                                       [0, 0, 1, 0.1],   # Vertical offset
                                       [0, 0, 0, 1]]))

    # Elbow coordinate system (relative to shoulder)
    elbow = CoordinateSystem("elbow",
                           np.array([[1, 0, 0, 0.3],   # Upper arm length
                                    [0, 1, 0, 0],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]]))

    # Wrist coordinate system (relative to elbow)
    wrist = CoordinateSystem("wrist",
                           np.array([[1, 0, 0, 0.25],  # Forearm length
                                    [0, 1, 0, 0],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]]))

    return {
        "world": world,
        "torso": torso,
        "shoulder": shoulder,
        "elbow": elbow,
        "wrist": wrist
    }

# Example usage: Transform a point from wrist to world coordinates
def example_transform_chain():
    cs = create_humanoid_arm_coordinate_systems()

    # Start with a point at the wrist
    wrist_local_point = np.array([0.05, 0, 0])  # Point 5cm forward from wrist

    # Chain transformations: wrist -> elbow -> shoulder -> torso -> world
    # Note: We need to multiply by each transform matrix in reverse order
    # because we're going from the innermost to outermost coordinate system

    # Calculate wrist to world transformation
    wrist_to_elbow = cs["elbow"].transform
    elbow_to_shoulder = cs["shoulder"].transform
    shoulder_to_torso = cs["torso"].transform
    torso_to_world = cs["world"].transform

    # Combine transformations (in reverse order for our example)
    # In practice, we'd calculate the cumulative transform properly
    print(f"Wrist position in world coordinates:")
    print(f"Shoulder: {cs['shoulder'].get_position()}")
    print(f"Elbow: {cs['shoulder'].transform[:3,3] + cs['elbow'].get_position()}")
    print(f"World origin: {cs['world'].get_position()}")

# Demonstrate coordinate system transformations
example_transform_chain()

# Show how to create transformations for specific movements
def create_hand_pose(shoulder_angles, elbow_angle, wrist_angles):
    """
    Create a coordinate system representing hand pose given joint angles

    Args:
        shoulder_angles: (z_rot, y_rot, x_rot) for shoulder
        elbow_angle: rotation around elbow joint
        wrist_angles: (z_rot, y_rot, x_rot) for wrist
    """
    # Create base transformation from joint angles
    hand_cs = CoordinateSystem("hand")

    # Apply rotations in sequence (order matters!)
    hand_cs.rotate_z(wrist_angles[0]).rotate_y(wrist_angles[1]).rotate_x(wrist_angles[2])

    # Translation along arm (simplified)
    hand_cs.translate(0.25, 0, 0)  # Move along forearm

    # Apply elbow rotation
    elbow_transform = CoordinateSystem("temp").rotate_z(elbow_angle)
    hand_cs.transform = elbow_transform.transform @ hand_cs.transform
    hand_cs.translate(0.3, 0, 0)  # Move along upper arm

    # Apply shoulder rotations
    shoulder_transform = (CoordinateSystem("temp")
                         .rotate_z(shoulder_angles[0])
                         .rotate_y(shoulder_angles[1])
                         .rotate_x(shoulder_angles[2]))
    hand_cs.transform = shoulder_transform.transform @ hand_cs.transform

    return hand_cs

# Example: Create a specific hand pose
pose = create_hand_pose(
    shoulder_angles=(np.pi/6, np.pi/8, np.pi/12),  # Shoulder: ~30°, 22.5°, 15°
    elbow_angle=np.pi/4,                            # Elbow: 45° flexion
    wrist_angles=(np.pi/6, 0, 0)                   # Wrist: 30° rotation
)

print(f"\nHand pose transformation matrix:")
print(pose.transform)
```

## Transformation Chain in Humanoid Robots

For humanoid robots, coordinate transformations follow the kinematic chain:

```
World → Base → Hip → Knee → Ankle → Foot
World → Base → Shoulder → Elbow → Wrist → Hand
World → Base → Torso → Neck → Head
```

Each transformation accumulates to provide the complete spatial relationship from world coordinates to any specific body part.

## Denavit-Hartenberg Convention

The DH convention provides a systematic way to define coordinate frames for serial manipulators:

1. **Z-axis**: Along the joint axis of motion
2. **X-axis**: Along the common normal between current and next joint axes
3. **Y-axis**: Completes the right-handed coordinate system

Parameters: θ (joint angle), d (link offset), a (link length), α (link twist)

## Best Practices

### Consistency
- Use consistent coordinate system definitions throughout the robot
- Follow established conventions (like DH parameters) when possible
- Document coordinate frame definitions clearly

### Verification
- Double-check transformation directions (which frame to which)
- Verify units (degrees vs radians for angles)
- Test transformations with known configurations

### Computational Efficiency
- Cache frequently used transformation matrices
- Use appropriate numerical precision for your application
- Consider quaternions for rotation representation to avoid gimbal lock

## Exercise

A humanoid robot's left arm has the following joint configuration:
- Shoulder at position (0.2, 0.15, 1.3) in world coordinates
- Upper arm length: 0.3 m
- Forearm length: 0.25 m
- Shoulder joint angles: (π/6, π/4, π/8) for ZYX rotations
- Elbow angle: π/3
- Wrist angles: (π/6, π/12, π/18)

Calculate the position and orientation of the robot's left hand in world coordinates using coordinate system transformations.

## Summary

Coordinate systems form the mathematical foundation for spatial reasoning in humanoid robotics. Proper understanding and implementation of transformations enable accurate robot control and interaction with the environment.
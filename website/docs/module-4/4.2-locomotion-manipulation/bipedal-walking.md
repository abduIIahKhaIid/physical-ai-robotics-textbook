---
title: Bipedal Locomotion Control
sidebar_position: 1
description: Advanced biomechanics and control systems for stable human-like walking in humanoid robotics
---

# Bipedal Walking

Bipedal walking is one of the most challenging aspects of humanoid robotics, requiring sophisticated control algorithms to achieve stable, efficient, and natural locomotion. Unlike wheeled or tracked robots, bipedal robots must manage complex balance and dynamic stability while adapting to terrain variations.

## Understanding Bipedal Locomotion

### The Challenge
Bipedal walking is inherently unstable because the robot stands on two narrow support points. Maintaining balance requires continuous adjustment of the center of mass and precise control of joint torques.

### Key Components
- **Balance Control**: Maintaining center of mass within support polygon
- **Gait Generation**: Coordinated movement of legs and body
- **Terrain Adaptation**: Adjusting for uneven surfaces and obstacles
- **Stability Management**: Recovering from disturbances

## Physics of Walking

### Center of Mass (CoM)
The center of mass must be kept within the support polygon (area between feet) for static stability or controlled for dynamic stability during walking.

### Zero Moment Point (ZMP)
ZMP is a critical concept in bipedal robotics. It's the point where the ground reaction force acts such that the moment about that point is zero. For stable walking, the ZMP must remain within the support polygon.

### Inverted Pendulum Model
The simplest model of bipedal walking treats the robot as an inverted pendulum. The CoM acts as the pendulum's mass, with the stance foot as the pivot point.

## Walking Phases

### Single Support Phase
- Only one foot is in contact with the ground
- Swing leg moves forward to prepare for next step
- Balance is dynamically maintained through momentum

### Double Support Phase
- Both feet are in contact with the ground
- Weight transfers from stance leg to swing leg
- Provides opportunity for balance recovery

### Flight Phase
- In some walking styles (dynamic walking), both feet leave the ground briefly
- Similar to running, but with controlled flight periods

## Gait Control Strategies

### Passive Dynamic Walking
- Uses gravity and momentum to achieve energy-efficient walking
- Minimal active control, similar to a child's wooden walking toy
- Very stable but limited adaptability

### Controlled Dynamic Walking
- Actively controls balance and step placement
- Uses feedback from sensors (IMU, force sensors, encoders)
- Most common approach for humanoid robots

### Capturability-Based Control
- Plans footsteps based on the robot's momentum state
- Ensures that the robot can come to a stop within a finite number of steps
- Uses Capture Point theory for step planning

## Mathematical Modeling

### Linear Inverted Pendulum Model (LIPM)
The LIPM simplifies the robot to a point mass at height h with no angular momentum:

**ẍ = g/h (x - zmp)**

Where:
- x is the center of mass position
- zmp is the zero moment point
- g is gravitational acceleration
- h is the center of mass height

### Preview Control
Preview control uses future ZMP references to calculate stable CoM trajectories:

**x(t) = x_ref(t) + A·exp(√(g/h)·t) + B·exp(-√(g/h)·t)**

## Implementation Approaches

### Pattern Generators
- Generate rhythmic walking patterns for joint trajectories
- Can be based on Central Pattern Generators (CPGs) or predefined functions
- Provide coordinated movement for multiple joints

### Model Predictive Control (MPC)
- Predicts future states based on current control actions
- Optimizes control inputs over a prediction horizon
- Accounts for constraints and disturbances

### Spring-Loaded Inverted Pendulum (SLIP)
- Models walking as a spring-mass system
- Mimics biological walking patterns
- Energy-efficient and robust to perturbations

## Control Architecture

```
High-Level Planner → Footstep Planner → Gait Generator → Joint Controllers
```

1. **Path Planning**: Determines overall walking trajectory
2. **Footstep Planning**: Plans where to place feet
3. **Trajectory Generation**: Creates CoM and ZMP trajectories
4. **Joint Level**: Executes joint position/force commands

## Implementation Example

```python
import numpy as np
from scipy.integrate import odeint

class BipedalWalker:
    def __init__(self, mass=75, height=1.0, com_height=0.85):
        self.mass = mass  # Robot mass in kg
        self.height = height  # Robot height in meters
        self.com_height = com_height  # Center of mass height
        self.g = 9.81  # Gravity
        self.omega = np.sqrt(self.g / self.com_height)  # LIPM frequency

        # Walking parameters
        self.step_length = 0.3  # Desired step length in meters
        self.step_width = 0.2   # Distance between feet in meters
        self.step_time = 0.8    # Time per step in seconds

        # State variables
        self.com_pos = np.array([0.0, 0.0, self.com_height])  # CoM position
        self.com_vel = np.array([0.0, 0.0, 0.0])             # CoM velocity

        # Foot positions (left and right)
        self.left_foot = np.array([-0.1, self.step_width/2, 0.0])
        self.right_foot = np.array([-0.1, -self.step_width/2, 0.0])

        self.current_support_foot = "left"  # Current stance foot

    def linear_inverted_pendulum_model(self, y, t, zmp_x, zmp_y):
        """
        Solve Linear Inverted Pendulum Model

        Args:
            y: State vector [x, vx, y, vy] where x,y are CoM pos and vx,vy are CoM vel
            t: Time
            zmp_x, zmp_y: Desired ZMP position

        Returns:
            Rate of change of state vector
        """
        x, vx, y, vy = y

        # LIPM dynamics: ddot_x = omega^2 * (x - zmp)
        ddt_x = self.omega**2 * (x - zmp_x)
        ddt_y = self.omega**2 * (y - zmp_y)

        return [vx, ddt_x, vy, ddt_y]

    def generate_step_trajectory(self, support_foot_pos, swing_foot_start,
                                swing_foot_end, step_phase):
        """
        Generate trajectory for swing foot during step

        Args:
            support_foot_pos: Position of stance foot
            swing_foot_start: Initial swing foot position
            swing_foot_end: Target swing foot position
            step_phase: Normalized phase (0.0 to 1.0) of current step

        Returns:
            numpy.ndarray: Swing foot position
        """
        # Calculate parabolic trajectory for foot lift
        lift_height = 0.1  # Maximum lift height
        progress = min(1.0, max(0.0, step_phase))

        # Horizontal interpolation
        x = swing_foot_start[0] + (swing_foot_end[0] - swing_foot_start[0]) * progress
        y = swing_foot_start[1] + (swing_foot_end[1] - swing_foot_start[1]) * progress

        # Vertical trajectory (parabolic lift)
        if progress < 0.5:
            # Rising phase
            z = swing_foot_start[2] + (lift_height * (progress / 0.5)**2)
        else:
            # Falling phase
            adjusted_progress = (progress - 0.5) / 0.5
            z = swing_foot_start[2] + lift_height - (lift_height * adjusted_progress**2)

        return np.array([x, y, z])

    def calculate_zmp_reference(self, com_trajectory, com_vel_trajectory):
        """
        Calculate ZMP from desired CoM trajectory

        Args:
            com_trajectory: Desired CoM positions
            com_vel_trajectory: Desired CoM velocities

        Returns:
            numpy.ndarray: ZMP positions
        """
        # ZMP = CoM - (h/g) * CoM_ddot
        # Approximate CoM_ddot from trajectory
        dt = 0.01  # Time step

        zmp_refs = []
        for i in range(len(com_trajectory)-1):
            # Velocity calculation
            com_vel_approx = (com_trajectory[i+1] - com_trajectory[i]) / dt
            # Acceleration approximation
            if i > 0:
                prev_vel = (com_trajectory[i] - com_trajectory[i-1]) / dt
                com_acc = (com_vel_approx - prev_vel) / dt
            else:
                com_acc = np.zeros(3)

            # ZMP calculation (only x,y matter for horizontal movement)
            zmp_x = com_trajectory[i][0] - (self.com_height / self.g) * com_acc[0]
            zmp_y = com_trajectory[i][1] - (self.com_height / self.g) * com_acc[1]

            zmp_refs.append([zmp_x, zmp_y])

        return np.array(zmp_refs)

    def walk_cycle_simulation(self, num_steps=5, dt=0.01):
        """
        Simulate a walking cycle using LIPM control

        Args:
            num_steps: Number of steps to simulate
            dt: Time step for simulation

        Returns:
            dict: Simulation results
        """
        # Generate walking pattern
        times = []
        com_positions = []
        zmp_positions = []
        support_feet = []

        current_time = 0
        com_state = [0.0, 0.0, 0.0, 0.0]  # [x, vx, y, vy]

        for step in range(num_steps):
            # Determine step destination
            if self.current_support_foot == "left":
                swing_foot_start = self.right_foot.copy()
                # Next step position (move forward and to left side)
                self.right_foot[0] = self.left_foot[0] + self.step_length
                self.right_foot[1] = -self.step_width / 2
                swing_foot_end = self.right_foot.copy()
            else:
                swing_foot_start = self.left_foot.copy()
                # Next step position (move forward and to right side)
                self.left_foot[0] = self.right_foot[0] + self.step_length
                self.left_foot[1] = self.step_width / 2
                swing_foot_end = self.left_foot.copy()

            # Simulate step duration
            step_duration = self.step_time
            step_steps = int(step_duration / dt)

            for i in range(step_steps):
                # Calculate current ZMP (simple: keep near stance foot)
                zmp_target = self.left_foot if self.current_support_foot == "left" else self.right_foot

                # Solve LIPM for CoM trajectory
                com_solution = odeint(
                    self.linear_inverted_pendulum_model,
                    com_state,
                    [current_time, current_time + dt],
                    args=(zmp_target[0], zmp_target[1])
                )

                # Update state
                com_state = com_solution[-1].tolist()

                # Record data
                times.append(current_time)
                com_positions.append([com_state[0], com_state[2], self.com_height])
                zmp_positions.append([zmp_target[0], zmp_target[1]])
                support_feet.append(self.current_support_foot)

                current_time += dt

            # Switch support foot
            self.current_support_foot = "right" if self.current_support_foot == "left" else "left"

        return {
            'times': times,
            'com_positions': np.array(com_positions),
            'zmp_positions': np.array(zmp_positions),
            'support_feet': support_feet
        }

# Example usage: Simulate walking
walker = BipedalWalker()
walk_data = walker.walk_cycle_simulation(num_steps=3)

print(f"Simulated {len(walk_data['times'])} time steps")
print(f"Starting CoM position: {walk_data['com_positions'][0]}")
print(f"Final CoM position: {walk_data['com_positions'][-1]}")
```

## Stability Considerations

### Capture Point Theory
Capture Point indicates where a robot must step to come to a complete stop. It's calculated as:

**Capture Point = CoM Position + (CoM Velocity / ω)**

Where ω = √(g/h) is the natural frequency of the inverted pendulum.

### Balance Recovery
When disturbed, robots must:
1. Detect the disturbance through IMU and force sensors
2. Determine if current trajectory is stable
3. Adjust either foot placement or CoM trajectory
4. Execute recovery actions quickly

## Practical Implementation Challenges

### Sensor Fusion
Combine data from multiple sensors (IMU, force sensors, encoders) to accurately estimate state.

### Ground Contact Detection
Accurately detect when feet make and break contact with the ground.

### Terrain Adaptation
Adjust walking parameters for different surfaces and slopes.

### Energy Efficiency
Minimize power consumption while maintaining stable walking.

## Exercise

Implement a simple ZMP-based walking controller for a simulated humanoid robot with the following specifications:
- Robot mass: 60 kg
- CoM height: 0.8 m
- Desired walking speed: 0.5 m/s
- Step length: 0.3 m
- Step width: 0.2 m

Design the controller to maintain stable walking despite small disturbances. Use the LIPM model and incorporate basic balance recovery mechanisms.

## Summary

Bipedal walking combines complex dynamics, control theory, and mechanical engineering. Success requires understanding the physics of balance, implementing appropriate control algorithms, and accounting for real-world uncertainties and disturbances.
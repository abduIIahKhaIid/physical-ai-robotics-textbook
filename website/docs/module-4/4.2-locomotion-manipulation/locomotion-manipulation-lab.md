---
title: Integrated Locomotion and Manipulation Laboratory
sidebar_position: 5
description: Advanced laboratory exercises for coordinated locomotion and manipulation in humanoid robotics
---

# Locomotion and Manipulation Lab

This lab provides hands-on exercises to develop and test locomotion and manipulation skills in humanoid robots. Through these exercises, you'll practice implementing walking controllers, grasping algorithms, and coordinated manipulation while addressing real-world challenges.

## Lab Objectives

After completing this lab, you will be able to:
- Implement basic walking controllers using different gait patterns
- Design and execute grasp planning algorithms
- Coordinate locomotion and manipulation for complex tasks
- Address practical challenges in humanoid robot control

## Exercise 1: Simple Walking Controller

### Setup
Implement a basic walking controller for a simplified humanoid robot model.

### Requirements
- Robot has 6 DOF legs (hip pitch/roll/yaw, knee, ankle pitch/roll)
- Walking speed: 0.5 m/s
- Step length: 0.3 m
- Step width: 0.2 m
- Stable ZMP tracking

### Tasks
1. **Gait Generation**: Create a pattern generator for basic walking
2. **Balance Control**: Implement simple ZMP-based balance control
3. **Trajectory Following**: Generate and track foot trajectories
4. **Stability Testing**: Verify stability under small disturbances

### Implementation Template
```python
import numpy as np
import matplotlib.pyplot as plt

class SimpleWalkingController:
    def __init__(self):
        # Robot parameters
        self.com_height = 0.8  # Center of mass height
        self.step_length = 0.3
        self.step_width = 0.2
        self.step_time = 0.8
        self.g = 9.81
        self.omega = np.sqrt(self.g / self.com_height)

        # State variables
        self.current_time = 0
        self.left_support = True  # Start with left foot support
        self.com_x = 0.0
        self.com_y = 0.0
        self.com_z = self.com_height

        # Foot positions (relative to CoM)
        self.left_foot = np.array([0.0, self.step_width/2, 0.0])
        self.right_foot = np.array([0.0, -self.step_width/2, 0.0])

    def update_gait_phase(self, dt):
        """Update the current phase in the walking cycle"""
        self.current_time += dt
        phase = (self.current_time % self.step_time) / self.step_time
        return phase

    def calculate_foot_trajectory(self, support_foot, swing_foot_start, swing_foot_target, phase):
        """Generate trajectory for the swing foot"""
        # Implement foot trajectory generation
        # Hint: Use polynomial or trigonometric interpolation
        pass

    def step(self, dt):
        """Perform one control step"""
        phase = self.update_gait_phase(dt)

        # Determine which foot is swing foot
        if self.left_support:
            support_foot = self.left_foot
            swing_foot = self.right_foot
            swing_target = np.array([self.left_foot[0] + self.step_length,
                                   -self.step_width/2, 0.0])
        else:
            support_foot = self.right_foot
            swing_foot = self.left_foot
            swing_target = np.array([self.right_foot[0] + self.step_length,
                                   self.step_width/2, 0.0])

        # Generate swing foot trajectory
        # TO BE IMPLEMENTED
        swing_pos = self.calculate_foot_trajectory(support_foot, swing_foot,
                                                 swing_target, phase)

        # Update support foot (it moves forward after step)
        if phase > 0.9:  # At end of step
            if self.left_support:
                self.left_foot[0] += self.step_length
            else:
                self.right_foot[0] += self.step_length

            # Switch support foot
            self.left_support = not self.left_support

        # Return current state for visualization
        return {
            'com': [self.com_x, self.com_y, self.com_z],
            'left_foot': self.left_foot.copy(),
            'right_foot': self.right_foot.copy()
        }

# Test the walking controller
controller = SimpleWalkingController()
dt = 0.01
steps = []
for i in range(500):  # Simulate 5 seconds
    state = controller.step(dt)
    steps.append(state)
    if i % 100 == 0:  # Print every second
        print(f"Step {i}: CoM at {state['com'][:2]}")

# Visualize the walk
if len(steps) > 0:
    com_x = [s['com'][0] for s in steps]
    com_y = [s['com'][1] for s in steps]
    left_x = [s['left_foot'][0] for s in steps]
    left_y = [s['left_foot'][1] for s in steps]
    right_x = [s['right_foot'][0] for s in steps]
    right_y = [s['right_foot'][1] for s in steps]

    plt.figure(figsize=(12, 5))
    plt.subplot(1, 2, 1)
    plt.plot(com_x, com_y, 'r-', label='CoM trajectory', linewidth=2)
    plt.scatter(left_x, left_y, c='blue', s=20, alpha=0.6, label='Left foot')
    plt.scatter(right_x, right_y, c='green', s=20, alpha=0.6, label='Right foot')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.title('Walking Pattern: X-Y Projection')

    plt.subplot(1, 2, 2)
    plt.plot([s['com'][0] for s in steps], label='CoM X')
    plt.plot([s['left_foot'][0] for s in steps], label='Left Foot X', linestyle='--')
    plt.plot([s['right_foot'][0] for s in steps], label='Right Foot X', linestyle='--')
    plt.xlabel('Time Step')
    plt.ylabel('X Position (m)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.title('Forward Progression')

    plt.tight_layout()
    plt.show()
```

### Exercise Instructions
Complete the `calculate_foot_trajectory` method to implement smooth foot trajectories for walking. Consider:
- Foot lift height for obstacle clearance
- Smooth transitions between phases
- Landing at the correct position

## Exercise 2: Grasp Planning and Execution

### Setup
Create a grasp planner that can select appropriate grasp types based on object properties.

### Requirements
- Classify objects as requiring power grasp or precision grasp
- Generate joint angle trajectories for grasp execution
- Consider object orientation and approach direction
- Evaluate grasp stability using simple metrics

### Implementation Template
```python
class GraspPlannerLab:
    def __init__(self):
        self.finger_count = 5
        self.max_force = 20.0
        # Define finger lengths and positions (simplified)
        self.finger_lengths = [0.08, 0.07, 0.07, 0.07, 0.05]  # thumb to pinky
        self.finger_spread = [0.05, 0.04, 0.03, 0.03, 0.02]

    def classify_object_for_grasping(self, object_size, object_weight, object_fragility):
        """
        Classify object to determine appropriate grasp type

        Args:
            object_size: Size of the object (max dimension in meters)
            object_weight: Weight of the object in kg
            object_fragility: Fragility index (0-1, 0=durable, 1=fragile)

        Returns:
            str: 'power' or 'precision'
        """
        # TO BE IMPLEMENTED: Implement classification logic
        # Hints: Large/heavy objects → power grasp, small/light/fragile → precision
        if object_size > 0.1 or object_weight > 0.5 or object_fragility < 0.3:
            return 'power'
        else:
            return 'precision'

    def generate_grasp_configuration(self, object_pose, grasp_type):
        """
        Generate hand configuration for grasping

        Args:
            object_pose: Pose of the object [position (3), orientation (quaternion)]
            grasp_type: Type of grasp ('power' or 'precision')

        Returns:
            dict: Joint angles and grasp parameters
        """
        # TO BE IMPLEMENTED: Calculate approach direction and finger positions
        approach_direction = object_pose[0] - np.array([0.5, 0, 0.8])  # From default hand position
        approach_direction = approach_direction / np.linalg.norm(approach_direction)

        if grasp_type == 'power':
            # Wide grip, multiple finger contact
            finger_angles = [0.8, 0.6, 0.6, 0.6, 0.4]  # Bent fingers
            grasp_width = min(0.15, np.linalg.norm(object_pose[0]) * 0.5)
        else:  # precision
            # Fingertip contact
            finger_angles = [0.3, 0.3, 0.3, 0.3, 0.2]
            grasp_width = 0.05

        return {
            'finger_angles': finger_angles,
            'grasp_width': grasp_width,
            'approach_direction': approach_direction,
            'grasp_type': grasp_type
        }

    def execute_grasp_sequence(self, object_pose, grasp_config, dt=0.01):
        """
        Generate trajectory for executing the grasp

        Args:
            object_pose: Target object pose
            grasp_config: Config from generate_grasp_configuration
            dt: Time step

        Returns:
            list: Sequence of joint configurations over time
        """
        # TO BE IMPLEMENTED: Generate approach and closure sequence
        trajectory = []

        # Approach phase (2 seconds)
        approach_duration = 2.0
        approach_steps = int(approach_duration / dt)

        for i in range(approach_steps):
            progress = i / approach_steps
            # Interpolate hand position toward object
            start_pos = np.array([0.5, 0, 0.8])
            target_pos = object_pose[0]

            current_pos = start_pos + progress * (target_pos - start_pos)

            # Gradually adjust finger angles
            finger_interp = [ang * progress for ang in grasp_config['finger_angles']]

            trajectory.append({
                'position': current_pos,
                'orientation': object_pose[1],  # Same orientation as object
                'finger_angles': finger_interp,
                'phase': 'approach'
            })

        # Grasp closure phase (1 second)
        closure_duration = 1.0
        closure_steps = int(closure_duration / dt)

        for i in range(closure_steps):
            # Maintain position, close fingers
            final_config = trajectory[-1].copy()
            final_config['phase'] = 'closure'

            # Interpolate to final finger positions
            closure_progress = i / closure_steps
            for j in range(len(final_config['finger_angles'])):
                start_angle = trajectory[approach_steps-1]['finger_angles'][j]
                final_angle = grasp_config['finger_angles'][j]
                final_config['finger_angles'][j] = start_angle + \
                    closure_progress * (final_angle - start_angle)

            trajectory.append(final_config)

        return trajectory

# Test the grasp planner
grasp_planner = GraspPlannerLab()

# Test objects
objects = [
    {'size': 0.15, 'weight': 1.2, 'fragility': 0.1, 'pose': [np.array([0.6, 0.1, 0.7]), np.array([0, 0, 0, 1])]},
    {'size': 0.03, 'weight': 0.05, 'fragility': 0.9, 'pose': [np.array([0.55, -0.1, 0.75]), np.array([0, 0, 0, 1])]},
    {'size': 0.08, 'weight': 0.3, 'fragility': 0.5, 'pose': [np.array([0.65, 0.05, 0.65]), np.array([0, 0, 0, 1])]}
]

for i, obj in enumerate(objects):
    grasp_type = grasp_planner.classify_object_for_grasping(
        obj['size'], obj['weight'], obj['fragility']
    )
    config = grasp_planner.generate_grasp_configuration(obj['pose'], grasp_type)
    trajectory = grasp_planner.execute_grasp_sequence(obj['pose'], config)

    print(f"Object {i+1}: Size={obj['size']:.2f}m, Weight={obj['weight']:.2f}kg, Fragile={obj['fragility']}")
    print(f"  Grasp type: {grasp_type}")
    print(f"  Grasp width: {config['grasp_width']:.3f}m")
    print(f"  Trajectory length: {len(trajectory)} steps")
    print()
```

### Exercise Instructions
Complete the `classify_object_for_grasping` and `generate_grasp_configuration` methods. Consider:
- How object size, weight, and fragility influence grasp selection
- How to position fingers appropriately for different grasp types
- How to generate smooth approach trajectories

## Exercise 3: Coordinated Locomotion-Manipulation Task

### Setup
Implement a task that requires the robot to walk to a location, pick up an object, and deliver it elsewhere.

### Requirements
- Navigate to pickup location
- Grasp specified object
- Walk to delivery location
- Release object
- Handle intermediate disturbances

### Implementation Template
```python
class CoordinatedTaskController:
    def __init__(self):
        self.walking_controller = SimpleWalkingController()
        self.grasp_planner = GraspPlannerLab()
        self.state = 'navigating_to_pickup'  # State machine
        self.target_object = None
        self.carried_object = None
        self.pickup_location = np.array([1.0, 0.0, 0.0])
        self.delivery_location = np.array([2.0, 0.5, 0.0])
        self.robot_position = np.array([0.0, 0.0, 0.0])

    def update(self, dt):
        """Main control loop for the coordinated task"""
        if self.state == 'navigating_to_pickup':
            # TO BE IMPLEMENTED: Navigate toward pickup location
            # Use walking controller to move robot
            # Switch to 'grasping' when near pickup location
            distance_to_pickup = np.linalg.norm(self.robot_position - self.pickup_location)
            if distance_to_pickup < 0.2:  # Close enough
                self.state = 'grasping'

        elif self.state == 'grasping':
            # TO BE IMPLEMENTED: Execute grasp sequence
            # Switch to 'navigating_to_delivery' when grasp successful
            # For now, simulate success after a few steps
            if np.random.random() < 0.1:  # 10% chance per step to complete
                self.carried_object = self.target_object
                self.state = 'navigating_to_delivery'

        elif self.state == 'navigating_to_delivery':
            # TO BE IMPLEMENTED: Navigate toward delivery location
            # Consider carrying object affects balance
            # Switch to 'releasing' when near delivery location
            distance_to_delivery = np.linalg.norm(self.robot_position - self.delivery_location)
            if distance_to_delivery < 0.2:  # Close enough
                self.state = 'releasing'

        elif self.state == 'releasing':
            # TO BE IMPLEMENTED: Release object
            # Switch to 'completed' when done
            if np.random.random() < 0.1:  # 10% chance per step to complete
                self.carried_object = None
                self.state = 'completed'

        # Update robot position based on walking controller
        walk_state = self.walking_controller.step(dt)
        self.robot_position[0] = walk_state['com'][0]
        self.robot_position[1] = walk_state['com'][1]

        return self.state

# Simulate the coordinated task
coordinator = CoordinatedTaskController()
coordinator.target_object = {'size': 0.1, 'weight': 0.2, 'fragility': 0.3}

dt = 0.1
max_steps = 500  # Maximum steps to complete task
current_state = coordinator.state

for step in range(max_steps):
    current_state = coordinator.update(dt)

    if current_state == 'completed':
        print(f"Task completed successfully in {step} steps!")
        break

    if step % 50 == 0:  # Print status every 5 seconds
        print(f"Step {step}: State = {current_state}, "
              f"Robot at [{coordinator.robot_position[0]:.2f}, "
              f"{coordinator.robot_position[1]:.2f}]")

if current_state != 'completed':
    print(f"Task timed out after {max_steps} steps. Final state: {current_state}")
```

### Exercise Instructions
Complete the `update` method to implement the full coordinated task. Consider:
- How to integrate walking and manipulation controllers
- How carrying an object affects locomotion
- How to handle disturbances or failures in either subsystem
- How to maintain task coordination when components operate at different frequencies

## Exercise 4: Disturbance Recovery

### Setup
Implement recovery mechanisms for when the robot experiences external disturbances.

### Tasks
1. **Disturbance Detection**: Identify when the robot is out of balance
2. **Recovery Actions**: Implement corrective actions for different disturbance types
3. **Stability Metrics**: Monitor key metrics to assess stability

### Implementation
```python
def check_balance_stability(robot_state):
    """
    Check if robot is maintaining balance

    Args:
        robot_state: Current state from walking controller

    Returns:
        dict: Stability assessment
    """
    # Calculate ZMP (simplified)
    com_pos = robot_state['com']
    com_vel = np.array([0.1, 0.05, 0])  # Assume small velocity
    com_acc = np.array([0.01, 0.005, 0])  # Assume small acceleration

    # ZMP calculation: ZMP = CoM - (h/g) * CoM_ddot
    zmp_x = com_pos[0] - (com_pos[2] / 9.81) * com_acc[0]
    zmp_y = com_pos[1] - (com_pos[2] / 9.81) * com_acc[1]

    # Determine support polygon (simplified as rectangle between feet)
    left_foot = robot_state['left_foot']
    right_foot = robot_state['right_foot']

    # Calculate bounds of support polygon
    x_min = min(left_foot[0], right_foot[0]) - 0.05  # Small margin
    x_max = max(left_foot[0], right_foot[0]) + 0.05
    y_min = min(left_foot[1], right_foot[1]) - 0.05
    y_max = max(left_foot[1], right_foot[1]) + 0.05

    # Check if ZMP is inside support polygon
    zmp_in_polygon = (x_min <= zmp_x <= x_max and y_min <= zmp_y <= y_max)

    # Calculate distance to nearest support edge
    zmp_margin = min(abs(zmp_x - x_min), abs(zmp_x - x_max),
                     abs(zmp_y - y_min), abs(zmp_y - y_max))

    return {
        'zmp_in_polygon': zmp_in_polygon,
        'zmp_position': (zmp_x, zmp_y),
        'margin_to_support': zmp_margin,
        'com_height': com_pos[2]
    }

# Test stability checker with different robot states
test_states = [
    {'com': [0.5, 0.0, 0.8], 'left_foot': [0.4, 0.1, 0.0], 'right_foot': [0.4, -0.1, 0.0]},  # Stable
    {'com': [0.7, 0.0, 0.8], 'left_foot': [0.4, 0.1, 0.0], 'right_foot': [0.4, -0.1, 0.0]},  # Unstable - CoM too far
    {'com': [0.5, 0.15, 0.8], 'left_foot': [0.4, 0.1, 0.0], 'right_foot': [0.4, -0.1, 0.0]}  # At boundary
]

print("Stability Analysis:")
for i, state in enumerate(test_states):
    stability = check_balance_stability(state)
    print(f"State {i+1}: ZMP in polygon = {stability['zmp_in_polygon']}, "
          f"Margin = {stability['margin_to_support']:.3f}m, "
          f"ZMP = ({stability['zmp_position'][0]:.3f}, {stability['zmp_position'][1]:.3f})")
```

### Exercise Instructions
Extend the stability checker to include:
- Proactive balance recovery when ZMP approaches support polygon boundary
- Different recovery strategies for various types of disturbances
- Integration with the walking controller to adjust gait parameters for improved stability

## Exercise 5: Real-world Considerations

### Tasks
1. **Sensor Noise**: Implement filtering and robustness to noisy measurements
2. **Actuator Limits**: Respect physical constraints on joint angles and velocities
3. **Timing Constraints**: Handle computation delays and real-time requirements
4. **Failure Modes**: Handle graceful degradation when subsystems fail

### Implementation Framework
```python
class RobustControlSystem:
    def __init__(self):
        self.noise_level = 0.01  # Measurement noise
        self.actuator_limits = {
            'max_velocity': 1.0,  # rad/s
            'max_torque': 50.0    # Nm
        }
        self.computation_delay = 0.02  # 20ms delay

    def add_sensor_noise(self, measurement):
        """Simulate sensor noise"""
        noise = np.random.normal(0, self.noise_level, measurement.shape)
        return measurement + noise

    def enforce_actuator_limits(self, desired_action):
        """Apply actuator constraints"""
        # Example: limit joint velocities
        if isinstance(desired_action, dict) and 'velocities' in desired_action:
            desired_action['velocities'] = np.clip(
                desired_action['velocities'],
                -self.actuator_limits['max_velocity'],
                self.actuator_limits['max_velocity']
            )
        return desired_action

    def handle_computation_delay(self, control_output):
        """Simulate delayed control action"""
        # In a real system, you'd implement a delay buffer
        # Here we just add a placeholder
        return control_output

# Demonstrate robust control concepts
robust_sys = RobustControlSystem()

# Simulate a noisy measurement
true_value = np.array([0.5, 0.0, 0.8])
measured_value = robust_sys.add_sensor_noise(true_value)

print(f"True CoM position: {true_value}")
print(f"Noisy measurement: {measured_value}")
print(f"Measurement error: {np.linalg.norm(true_value - measured_value):.3f}m")

# Simulate actuator limits
desired_action = {
    'velocities': np.array([1.5, 0.8, -1.2, 2.0])  # Some exceed limits
}
limited_action = robust_sys.enforce_actuator_limits(desired_action)
print(f"Desired velocities: {desired_action['velocities']}")
print(f"Limited velocities: {limited_action['velocities']}")
```

## Assessment Questions

### Conceptual Understanding
1. How do gait parameters (step length, width, timing) affect walking stability?
2. What are the trade-offs between power and precision grasps?
3. How does carrying an object affect a robot's locomotion requirements?
4. What are the key factors that determine the success of coordinated tasks?

### Implementation Challenges
1. How would you modify the walking controller to handle sloped terrain?
2. How would you extend the grasp planner to handle unknown object properties?
3. What safety measures would you implement for real-world deployment?
4. How would you optimize the system for energy efficiency?

## Extension Activities

1. **Simulation Integration**: Connect your controllers to a physics simulator like PyBullet or MuJoCo
2. **Machine Learning**: Use reinforcement learning to optimize gait parameters
3. **Multi-Robot Coordination**: Extend to multiple robots collaborating on manipulation tasks
4. **Adaptive Control**: Implement online adaptation to changing conditions

## Summary

This lab has guided you through the fundamental aspects of locomotion and manipulation in humanoid robotics. From basic walking controllers to complex coordinated tasks, you've gained hands-on experience with the core challenges and techniques required for effective humanoid robot control. Continue experimenting with these concepts to deepen your understanding of this fascinating field.
---
title: Vision-Language-Action Experimental Framework
sidebar_position: 4
description: Laboratory exercises and experimental validation of Vision-Language-Action concepts in humanoid robotics
---

# VLA Mini-Experiments

This lab provides hands-on experiments to explore Vision-Language-Action (VLA) concepts using simplified implementations. These experiments help develop intuition about how vision, language, and action can be integrated in humanoid robotics systems.

## Lab Objectives

After completing these experiments, you will be able to:
- Understand the relationship between vision, language, and action in robotics
- Implement basic VLA components using available tools
- Evaluate VLA system performance and limitations
- Apply VLA concepts to practical scenarios

## Experiment 1: Simple Object Recognition and Action Mapping

### Goal
Create a system that recognizes objects in an image and suggests appropriate actions based on natural language commands.

### Setup
We'll use a simplified object detection system and map recognized objects to actions.

```python
import numpy as np
import cv2
from PIL import Image
import matplotlib.pyplot as plt

class SimpleVLASystem:
    def __init__(self):
        # Predefined object-action mappings
        self.object_actions = {
            'cup': ['grasp', 'lift', 'carry', 'place'],
            'ball': ['grasp', 'throw', 'bounce', 'catch'],
            'book': ['grasp', 'open', 'turn_page', 'close', 'stack'],
            'bottle': ['grasp', 'pour', 'fill', 'cap'],
            'box': ['grasp', 'open', 'pack', 'unpack', 'stack'],
            'phone': ['grasp', 'answer', 'dial', 'charge']
        }

        # Predefined location descriptions
        self.locations = {
            'kitchen': ['counter', 'sink', 'fridge', 'table'],
            'living_room': ['sofa', 'coffee_table', 'tv', 'bookshelf'],
            'bedroom': ['bed', 'dresser', 'nightstand', 'closet']
        }

    def detect_objects_simple(self, image):
        """
        Simple object detection using predefined object locations
        (In practice, this would use a trained object detector)
        """
        # Simulate object detection on a simple scene
        objects = []

        # For demonstration, return mock detections
        if len(image.shape) == 3:  # Color image
            height, width = image.shape[:2]

            # Add some mock objects with random positions
            if np.random.rand() > 0.3:  # 70% chance of cup
                objects.append({
                    'class': 'cup',
                    'bbox': [int(width*0.3), int(height*0.4), int(width*0.4), int(height*0.5)],
                    'confidence': 0.85
                })

            if np.random.rand() > 0.5:  # 50% chance of book
                objects.append({
                    'class': 'book',
                    'bbox': [int(width*0.6), int(height*0.3), int(width*0.7), int(height*0.6)],
                    'confidence': 0.78
                })

        return objects

    def parse_command(self, command):
        """
        Simple command parsing to extract intent
        """
        command_lower = command.lower()

        # Extract action intent
        if 'pick up' in command_lower or 'grasp' in command_lower:
            intent = 'grasp'
        elif 'put' in command_lower or 'place' in command_lower:
            intent = 'place'
        elif 'move' in command_lower:
            intent = 'move'
        elif 'pour' in command_lower:
            intent = 'pour'
        else:
            intent = 'unknown'

        # Extract object reference
        for obj_class in self.object_actions.keys():
            if obj_class in command_lower:
                target_object = obj_class
                break
        else:
            target_object = 'unknown'

        return {
            'intent': intent,
            'target_object': target_object,
            'original_command': command
        }

    def generate_action_sequence(self, detected_objects, parsed_command):
        """
        Generate action sequence based on detected objects and command
        """
        # Find the target object among detections
        target_obj = None
        for obj in detected_objects:
            if obj['class'] == parsed_command['target_object']:
                target_obj = obj
                break

        if not target_obj:
            # If exact object not found, try to find a suitable alternative
            for obj in detected_objects:
                if obj['class'] in self.object_actions:
                    target_obj = obj
                    break

        if not target_obj:
            return {
                'action_sequence': [],
                'status': 'no_suitable_object_found',
                'message': 'No suitable object found for the command.'
            }

        # Get available actions for this object
        available_actions = self.object_actions.get(target_obj['class'], [])

        # Determine the best action based on command intent
        if parsed_command['intent'] in available_actions:
            selected_action = parsed_command['intent']
        elif parsed_command['intent'] == 'grasp' and 'grasp' in available_actions:
            selected_action = 'grasp'
        else:
            # Default to first available action
            selected_action = available_actions[0] if available_actions else 'idle'

        # Generate action sequence
        action_sequence = []

        if selected_action == 'grasp':
            action_sequence = [
                {'action': 'approach_object', 'target': target_obj['bbox']},
                {'action': 'align_hand', 'object_bbox': target_obj['bbox']},
                {'action': 'grasp', 'object_class': target_obj['class']},
                {'action': 'lift_slightly', 'height': 0.1}
            ]
        elif selected_action == 'place':
            action_sequence = [
                {'action': 'navigate_to_target', 'location': 'table'},
                {'action': 'lower_object', 'height': 0.05},
                {'action': 'release_grip', 'object_class': target_obj['class']},
                {'action': 'withdraw_hand', 'distance': 0.1}
            ]
        elif selected_action == 'move':
            action_sequence = [
                {'action': 'grasp', 'object_class': target_obj['class']},
                {'action': 'lift', 'height': 0.2},
                {'action': 'navigate_to_target', 'location': 'destination'},
                {'action': 'place', 'object_class': target_obj['class']}
            ]

        return {
            'action_sequence': action_sequence,
            'target_object': target_obj,
            'selected_action': selected_action,
            'status': 'success'
        }

    def visualize_result(self, image, detected_objects, action_sequence):
        """
        Visualize the VLA system's interpretation
        """
        vis_image = image.copy()

        # Draw bounding boxes
        for obj in detected_objects:
            bbox = obj['bbox']
            x1, y1, x2, y2 = bbox
            cv2.rectangle(vis_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(vis_image, f"{obj['class']} ({obj['confidence']:.2f})",
                       (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Display action sequence as text
        y_offset = 20
        for i, action in enumerate(action_sequence['action_sequence']):
            cv2.putText(vis_image, f"Action {i+1}: {action['action']}",
                       (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
            y_offset += 20

        return vis_image

# Create and test the VLA system
vla_system = SimpleVLASystem()

# Create a sample image (in practice, this would come from a camera)
sample_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

# Add some simple "objects" to the image
cv2.rectangle(sample_image, (150, 200, 200, 300), (255, 0, 0), -1)  # Red rectangle
cv2.rectangle(sample_image, (300, 150, 400, 250), (0, 255, 0), -1)  # Green rectangle

# Test with different commands
test_commands = [
    "Pick up the red cup",
    "Grasp the green book",
    "Move the red object to the table",
    "Place the item gently"
]

print("VLA System Test Results:")
print("="*50)

for i, command in enumerate(test_commands):
    print(f"\nTest {i+1}: Command = '{command}'")

    # Detect objects in image
    detected_objects = vla_system.detect_objects_simple(sample_image)
    print(f"Detected {len(detected_objects)} objects")

    # Parse the command
    parsed = vla_system.parse_command(command)
    print(f"Parsed intent: {parsed['intent']}, target: {parsed['target_object']}")

    # Generate action sequence
    result = vla_system.generate_action_sequence(detected_objects, parsed)
    print(f"Action sequence: {[action['action'] for action in result['action_sequence']]}")

    # Visualize (just show structure for this demo)
    print(f"Target object: {result.get('target_object', {}).get('class', 'None')}")
    print(f"Status: {result['status']}")

print("\nExperiment 1 completed.")
```

## Experiment 2: Language-Guided Manipulation

### Goal
Implement a system that can interpret natural language instructions to perform simple manipulation tasks.

### Setup
Create a manipulation planner that translates language commands into robot actions.

```python
class LanguageGuidedManipulation:
    def __init__(self):
        # Define basic manipulation actions
        self.manipulation_actions = {
            'grasp': {
                'prerequisites': ['object_detected'],
                'effects': ['object_grasped'],
                'description': 'Grasp an object at the specified location'
            },
            'release': {
                'prerequisites': ['object_grasped'],
                'effects': ['object_released'],
                'description': 'Release the currently held object'
            },
            'move_to': {
                'prerequisites': ['nav_capability'],
                'effects': ['location_changed'],
                'description': 'Move the robot to a specified location'
            },
            'pour': {
                'prerequisites': ['container_grasped', 'pour_target_identified'],
                'effects': ['liquid_transferred'],
                'description': 'Pour contents from one container to another'
            },
            'stack': {
                'prerequisites': ['object_grasped', 'stacking_area_valid'],
                'effects': ['object_placed_on_stack'],
                'description': 'Stack object on top of existing stack'
            }
        }

        # Define manipulation primitives
        self.primitives = {
            'reach_to_position': {'params': ['x', 'y', 'z']},
            'adjust_gripper': {'params': ['opening_width']},
            'apply_force': {'params': ['magnitude', 'direction']},
            'monitor_force': {'params': ['threshold', 'duration']},
            'track_object': {'params': ['object_id']}
        }

        # Task planning vocabulary
        self.task_vocab = {
            'actions': ['pick', 'place', 'move', 'grasp', 'release', 'pour', 'stack', 'organize'],
            'objects': ['cup', 'bowl', 'plate', 'book', 'pen', 'bottle', 'box'],
            'locations': ['table', 'shelf', 'counter', 'drawer', 'cabinet', 'floor'],
            'modifiers': ['gently', 'carefully', 'quickly', 'precisely']
        }

    def parse_manipulation_command(self, command):
        """
        Parse manipulation command into structured representation
        """
        import re

        # Extract action verb
        action = None
        for act in self.task_vocab['actions']:
            if act in command.lower():
                action = act
                break

        # Extract object
        obj = None
        for o in self.task_vocab['objects']:
            if o in command.lower():
                obj = o
                break

        # Extract location
        loc = None
        for l in self.task_vocab['locations']:
            if l in command.lower():
                loc = l
                break

        # Extract modifiers
        mods = []
        for mod in self.task_vocab['modifiers']:
            if mod in command.lower():
                mods.append(mod)

        return {
            'action': action,
            'object': obj,
            'location': loc,
            'modifiers': mods,
            'original_command': command
        }

    def plan_manipulation_sequence(self, parsed_command, world_state):
        """
        Plan manipulation sequence based on command and world state
        """
        # Check if command is complete
        if not parsed_command['action']:
            return {
                'valid': False,
                'error': 'No action specified in command',
                'sequence': []
            }

        # Generate manipulation plan based on action type
        plan = []

        if parsed_command['action'] in ['pick', 'grasp']:
            plan = self._plan_pick_action(parsed_command, world_state)
        elif parsed_command['action'] in ['place', 'release']:
            plan = self._plan_place_action(parsed_command, world_state)
        elif parsed_command['action'] in ['move', 'transfer']:
            plan = self._plan_transfer_action(parsed_command, world_state)
        elif parsed_command['action'] == 'pour':
            plan = self._plan_pour_action(parsed_command, world_state)
        else:
            return {
                'valid': False,
                'error': f'Action "{parsed_command["action"]}" not supported',
                'sequence': []
            }

        return {
            'valid': True,
            'error': None,
            'sequence': plan,
            'command': parsed_command
        }

    def _plan_pick_action(self, command, world_state):
        """Plan pick/grasp action"""
        # Find object location in world state
        obj_location = world_state.get(f"{command['object']}_location")
        if not obj_location:
            # If object location not known, use default or request update
            obj_location = {'x': 0.5, 'y': 0.0, 'z': 0.8}  # Default position

        plan = [
            {
                'primitive': 'reach_to_position',
                'params': {'x': obj_location['x'], 'y': obj_location['y'], 'z': obj_location['z'] + 0.1},  # Above object
                'description': 'Approach object from above'
            },
            {
                'primitive': 'reach_to_position',
                'params': obj_location,
                'description': 'Position hand at object location'
            },
            {
                'primitive': 'adjust_gripper',
                'params': {'opening_width': 0.02},  # Grip slightly smaller than object
                'description': 'Close gripper around object'
            },
            {
                'primitive': 'monitor_force',
                'params': {'threshold': 5.0, 'duration': 0.1},
                'description': 'Confirm successful grasp through force feedback'
            },
            {
                'primitive': 'reach_to_position',
                'params': {'x': obj_location['x'], 'y': obj_location['y'], 'z': obj_location['z'] + 0.1},
                'description': 'Lift object slightly'
            }
        ]

        return plan

    def _plan_place_action(self, command, world_state):
        """Plan place/release action"""
        # Find destination location
        dest_location = world_state.get(f"{command['location']}_position")
        if not dest_location:
            # Default placement location
            dest_location = {'x': 0.6, 'y': 0.0, 'z': 0.8}

        plan = [
            {
                'primitive': 'reach_to_position',
                'params': {'x': dest_location['x'], 'y': dest_location['y'], 'z': dest_location['z'] + 0.1},  # Above destination
                'description': 'Navigate to destination location'
            },
            {
                'primitive': 'reach_to_position',
                'params': dest_location,
                'description': 'Position object at destination'
            },
            {
                'primitive': 'adjust_gripper',
                'params': {'opening_width': 0.05},  # Open gripper
                'description': 'Release object'
            },
            {
                'primitive': 'monitor_force',
                'params': {'threshold': 1.0, 'duration': 0.1},
                'description': 'Confirm object released'
            },
            {
                'primitive': 'reach_to_position',
                'params': {'x': dest_location['x'], 'y': dest_location['y'], 'z': dest_location['z'] + 0.1},
                'description': 'Withdraw hand'
            }
        ]

        return plan

    def _plan_transfer_action(self, command, world_state):
        """Plan transfer/move action (pick and place)"""
        # Combine pick and place plans
        pick_plan = self._plan_pick_action(command, world_state)
        place_plan = self._plan_place_action(command, world_state)

        return pick_plan + place_plan

    def _plan_pour_action(self, command, world_state):
        """Plan pouring action"""
        source_loc = world_state.get(f"{command['object']}_location", {'x': 0.5, 'y': 0.0, 'z': 0.8})
        dest_loc = world_state.get(f"{command['location']}_position", {'x': 0.6, 'y': 0.1, 'z': 0.8})

        plan = [
            {
                'primitive': 'reach_to_position',
                'params': source_loc,
                'description': 'Grasp the container to pour from'
            },
            {
                'primitive': 'adjust_gripper',
                'params': {'opening_width': 0.02},
                'description': 'Securely grip container'
            },
            {
                'primitive': 'reach_to_position',
                'params': {'x': (source_loc['x'] + dest_loc['x'])/2, 'y': (source_loc['y'] + dest_loc['y'])/2, 'z': source_loc['z']},
                'description': 'Move container above destination'
            },
            {
                'primitive': 'apply_force',
                'params': {'magnitude': 0.2, 'direction': 'tilt'},
                'description': 'Tilt container to pour'
            },
            {
                'primitive': 'apply_force',
                'params': {'magnitude': -0.2, 'direction': 'tilt'},
                'description': 'Return container to upright position'
            }
        ]

        return plan

    def execute_plan(self, plan, simulate=True):
        """
        Execute manipulation plan (simulation mode)
        """
        if not simulate:
            # In real execution, this would send commands to robot
            return "Plan sent to robot for execution"

        # Simulate plan execution
        print("\nExecuting manipulation plan:")
        for i, step in enumerate(plan):
            print(f"  Step {i+1}: {step['description']}")
            print(f"    Primitive: {step['primitive']}")
            print(f"    Parameters: {step['params']}")

        return f"Simulated execution of {len(plan)} action steps"

# Test the language-guided manipulation system
manip_system = LanguageGuidedManipulation()

# Simulated world state
world_state = {
    'cup_location': {'x': 0.5, 'y': 0.1, 'z': 0.8},
    'bowl_location': {'x': 0.4, 'y': -0.1, 'z': 0.8},
    'table_position': {'x': 0.6, 'y': 0.0, 'z': 0.8},
    'shelf_position': {'x': 0.7, 'y': 0.2, 'z': 1.0}
}

# Test commands
manip_commands = [
    "Pick up the cup",
    "Place the bowl on the table",
    "Move the cup from its current position to the shelf",
    "Pour water from the cup into the bowl"
]

print("\nLanguage-Guided Manipulation Tests:")
print("="*50)

for i, cmd in enumerate(manip_commands):
    print(f"\nTest {i+1}: '{cmd}'")

    # Parse command
    parsed = manip_system.parse_manipulation_command(cmd)
    print(f"Parsed: action='{parsed['action']}', object='{parsed['object']}', location='{parsed['location']}'")

    # Plan sequence
    plan_result = manip_system.plan_manipulation_sequence(parsed, world_state)

    if plan_result['valid']:
        print(f"Generated plan with {len(plan_result['sequence'])} steps")
        # Execute (simulate)
        result = manip_system.execute_plan(plan_result['sequence'], simulate=True)
    else:
        print(f"Error: {plan_result['error']}")

print("\nExperiment 2 completed.")
```

## Experiment 3: Vision-Action Coordination

### Goal
Create a system that coordinates visual perception with action execution for basic object manipulation.

### Setup
Implement visual servoing and coordination between perception and action systems.

```python
class VisionActionCoordinator:
    def __init__(self):
        # Vision processing parameters
        self.feature_detectors = {
            'color': {'enabled': True, 'params': {'hue_range': [20, 30], 'saturation': 0.5}},
            'shape': {'enabled': True, 'params': {'min_area': 100, 'aspect_ratio': [0.5, 2.0]}},
            'motion': {'enabled': True, 'params': {'min_displacement': 2, 'window_size': 10}}
        }

        # Action execution parameters
        self.action_params = {
            'precision': 0.01,  # 1cm precision
            'speed': 0.1,       # 10 cm/s movement
            'force_limit': 20.0 # 20N max force
        }

        # State tracking
        self.tracked_objects = {}
        self.robot_pose = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'orientation': [0, 0, 0, 1]}

        # Coordination strategies
        self.coordination_strategies = {
            'position_servo': self.position_servo,
            'force_guided': self.force_guided_manipulation,
            'hybrid_control': self.hybrid_visual_force_control
        }

    def detect_and_track_object(self, image, object_class):
        """
        Detect and track a specific object class in the image
        """
        # Simulate object detection and tracking
        np.random.seed(42)  # For reproducible results

        # Generate mock detection results
        if object_class == 'cup':
            detection = {
                'bbox': [int(image.shape[1]*0.4), int(image.shape[0]*0.3),
                        int(image.shape[1]*0.5), int(image.shape[0]*0.6)],
                'center': [int(image.shape[1]*0.45), int(image.shape[0]*0.45)],
                'size_pixels': 15000,
                'confidence': 0.89
            }
        else:
            detection = {
                'bbox': [int(image.shape[1]*0.6), int(image.shape[0]*0.2),
                        int(image.shape[1]*0.7), int(image.shape[0]*0.5)],
                'center': [int(image.shape[1]*0.65), int(image.shape[0]*0.35)],
                'size_pixels': 8000,
                'confidence': 0.76
            }

        # Convert pixel coordinates to world coordinates (simplified)
        detection['world_coords'] = self.pixel_to_world_coords(detection['center'])

        return detection

    def pixel_to_world_coords(self, pixel_coords):
        """
        Convert pixel coordinates to world coordinates
        (Simplified calibration-based conversion)
        """
        px, py = pixel_coords

        # Simplified camera model (in practice, use calibrated parameters)
        # Assuming camera at (0.5, 0, 1.2) looking downward
        camera_height = 1.2
        focal_length = 500  # pixels

        # Convert to world coordinates (simplified pinhole model)
        # This is a very simplified version - in practice, use full calibration
        scale_factor = camera_height / focal_length
        world_x = (px - 320) * scale_factor * 0.1  # Adjust scaling as needed
        world_y = (py - 240) * scale_factor * 0.1
        world_z = 0.0  # Assume object is on ground plane

        return {'x': 0.5 + world_x, 'y': world_y, 'z': world_z}

    def position_servo(self, target_pixel_pos, current_pixel_pos, max_move=10):
        """
        Simple position-based visual servoing
        """
        dx = target_pixel_pos[0] - current_pixel_pos[0]
        dy = target_pixel_pos[1] - current_pixel_pos[1]

        # Limit movement to avoid large jumps
        dx = np.clip(dx, -max_move, max_move)
        dy = np.clip(dy, -max_move, max_move)

        # Convert to appropriate action commands
        # In practice, this would convert pixel errors to joint/pose commands
        action_commands = {
            'delta_x': dx * 0.001,  # Scale to meters
            'delta_y': dy * 0.001,
            'adjustment': [dx, dy]
        }

        return action_commands

    def force_guided_manipulation(self, target_object, desired_force_profile):
        """
        Use force feedback to guide manipulation actions
        """
        # Simulate force feedback during manipulation
        force_feedback = {
            'contact_force': 5.0,  # Newtons
            'slip_detection': False,
            'gripper_pressure': 10.0  # kPa
        }

        # Adjust manipulation based on force feedback
        if force_feedback['contact_force'] > 15.0:
            # Too much force - reduce grip
            adjustment = {'grip_change': -0.001}
        elif force_feedback['contact_force'] < 2.0:
            # Not enough force - increase grip
            adjustment = {'grip_change': 0.001}
        else:
            # Appropriate force - maintain
            adjustment = {'grip_change': 0.0}

        return adjustment

    def hybrid_visual_force_control(self, vision_error, force_feedback):
        """
        Combine visual and force feedback for coordinated control
        """
        # Weight visual and force components
        visual_contribution = vision_error * 0.7
        force_contribution = (force_feedback['contact_force'] - 5.0) * 0.3

        combined_correction = visual_contribution + force_contribution

        return {
            'combined_error': combined_correction,
            'visual_component': visual_contribution,
            'force_component': force_contribution,
            'action_command': combined_correction * 0.001  # Scale to robot command
        }

    def coordinate_vision_action(self, image, command, strategy='position_servo'):
        """
        Main coordination function
        """
        # Parse command to determine target object
        target_object = 'cup' if 'cup' in command.lower() else 'box'

        # Detect object in current image
        detection = self.detect_and_track_object(image, target_object)

        # Get target pixel position (from command or predefined)
        if 'center' in command.lower():
            target_pixel = [image.shape[1] // 2, image.shape[0] // 2]  # Image center
        else:
            target_pixel = detection['center']  # Object location

        # Get current position (simplified - assume robot center corresponds to image center)
        current_pixel = [image.shape[1] // 2, image.shape[0] // 2]

        # Execute coordination strategy
        strategy_func = self.coordination_strategies[strategy]

        if strategy == 'position_servo':
            action = strategy_func(target_pixel, current_pixel)
        elif strategy == 'force_guided':
            action = strategy_func(target_object, {})
        elif strategy == 'hybrid_control':
            vision_error = np.sqrt((target_pixel[0] - current_pixel[0])**2 +
                                 (target_pixel[1] - current_pixel[1])**2)
            force_feedback = {'contact_force': 5.0, 'slip_detection': False}
            action = strategy_func(vision_error, force_feedback)
        else:
            action = {'error': 'Unknown strategy'}

        return {
            'detection': detection,
            'target_pixel': target_pixel,
            'current_pixel': current_pixel,
            'strategy': strategy,
            'action': action,
            'success': 'error' not in action
        }

    def continuous_coordination_loop(self, initial_image, command, num_iterations=5):
        """
        Simulate a continuous vision-action coordination loop
        """
        current_image = initial_image.copy()
        coordination_results = []

        for iteration in range(num_iterations):
            # Add some "movement" to simulate changing scene
            current_image = self.simulate_scene_change(current_image, iteration)

            # Coordinate vision and action
            result = self.coordinate_vision_action(
                current_image, command, strategy='position_servo'
            )

            coordination_results.append(result)

            print(f"Iteration {iteration + 1}: Object at pixel {result['detection']['center']}, "
                  f"Action: dx={result['action']['delta_x']:.4f}, dy={result['action']['delta_y']:.4f}")

        return coordination_results

    def simulate_scene_change(self, image, iteration):
        """
        Simulate gradual scene changes for continuous coordination
        """
        # Add small random shifts to simulate object movement
        shift_x = np.random.randint(-2, 3)
        shift_y = np.random.randint(-2, 3)

        # Apply shift to image (simplified simulation)
        # In practice, this would come from new camera images
        return image  # Return unchanged for this simulation

# Test the vision-action coordinator
coordination_system = VisionActionCoordinator()

# Create sample image
sample_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

# Test commands
coordination_commands = [
    "Move to center of cup",
    "Track the box and maintain position",
    "Align with object center"
]

print("\nVision-Action Coordination Tests:")
print("="*50)

for i, cmd in enumerate(coordination_commands):
    print(f"\nTest {i+1}: '{cmd}'")

    # Single coordination step
    result = coordination_system.coordinate_vision_action(sample_image, cmd)

    print(f"Detection: Object at {result['detection']['center']}")
    print(f"Target pixel: {result['target_pixel']}")
    print(f"Strategy: {result['strategy']}")
    print(f"Action: {result['action']}")
    print(f"Success: {result['success']}")

# Test continuous coordination
print(f"\nContinuous coordination test:")
continuous_results = coordination_system.continuous_coordination_loop(
    sample_image, "Track and follow the cup", num_iterations=3
)

print(f"\nCompleted {len(continuous_results)} coordination iterations")
print("Experiment 3 completed.")
```

## Experiment 4: Integration Challenge

### Goal
Combine all VLA components into a unified system that can handle complex tasks.

### Setup
Create an integrated VLA system that combines perception, language understanding, and action execution.

```python
class IntegratedVLASystem:
    def __init__(self):
        # Initialize components
        self.vision_system = SimpleVLASystem()
        self.language_system = LanguageGuidedManipulation()
        self.action_system = VisionActionCoordinator()

        # Integration parameters
        self.confidence_threshold = 0.7
        self.execution_timeout = 30.0  # seconds
        self.recovery_attempts = 3

        # Task execution state
        self.execution_state = {
            'current_task': None,
            'subtask_index': 0,
            'world_state': {},
            'execution_history': []
        }

    def integrated_process(self, command, image):
        """
        Integrated VLA processing pipeline
        """
        print(f"\nProcessing command: '{command}'")

        # Step 1: Vision processing
        print("Step 1: Processing visual input...")
        detected_objects = self.vision_system.detect_objects_simple(image)
        print(f"  Detected {len(detected_objects)} objects")

        # Step 2: Language processing
        print("Step 2: Parsing language command...")
        if 'manipulation' in command.lower():
            parsed_command = self.language_system.parse_manipulation_command(command)
        else:
            parsed_command = self.vision_system.parse_command(command)
        print(f"  Parsed: {parsed_command}")

        # Step 3: Action planning
        print("Step 3: Generating action plan...")
        if hasattr(self.language_system, 'plan_manipulation_sequence'):
            plan_result = self.language_system.plan_manipulation_sequence(
                parsed_command, self.execution_state['world_state']
            )
            action_sequence = plan_result.get('sequence', [])
        else:
            # Fallback to vision-based action generation
            action_sequence = self.vision_system.generate_action_sequence(
                detected_objects, parsed_command
            ).get('action_sequence', [])

        print(f"  Generated {len(action_sequence)} action steps")

        # Step 4: Execute with vision guidance
        print("Step 4: Executing with vision feedback...")
        execution_log = []

        for i, action in enumerate(action_sequence):
            print(f"  Executing action {i+1}/{len(action_sequence)}: {action.get('action', action.get('primitive', 'unknown'))}")

            # Log execution
            execution_log.append({
                'step': i+1,
                'action': action,
                'status': 'completed',
                'details': f'Step {i+1} of {len(action_sequence)}'
            })

        # Update world state based on execution
        self.update_world_state(parsed_command, action_sequence)

        return {
            'command': command,
            'detected_objects': detected_objects,
            'parsed_command': parsed_command,
            'action_sequence': action_sequence,
            'execution_log': execution_log,
            'world_state_update': self.execution_state['world_state']
        }

    def update_world_state(self, parsed_command, action_sequence):
        """
        Update internal world representation based on actions
        """
        # Update based on action effects
        for action in action_sequence:
            action_type = action.get('action', action.get('primitive', ''))

            if 'grasp' in action_type and parsed_command.get('object'):
                obj_name = parsed_command['object']
                self.execution_state['world_state'][f'{obj_name}_grasped'] = True
                self.execution_state['world_state'][f'{obj_name}_location'] = {'x': 0.5, 'y': 0.0, 'z': 1.0}  # Hand location
            elif 'place' in action_type and parsed_command.get('location'):
                loc_name = parsed_command['location']
                if parsed_command.get('object'):
                    obj_name = parsed_command['object']
                    self.execution_state['world_state'][f'{obj_name}_grasped'] = False
                    self.execution_state['world_state'][f'{obj_name}_location'] = self.execution_state['world_state'].get(f'{loc_name}_position', {'x': 0.6, 'y': 0.0, 'z': 0.8})

    def execute_complex_task(self, task_description):
        """
        Execute a complex multi-step task
        """
        # Break down complex task into subtasks
        subtasks = self.break_down_task(task_description)

        print(f"\nExecuting complex task with {len(subtasks)} subtasks:")

        task_results = []
        for i, subtask in enumerate(subtasks):
            print(f"\nSubtask {i+1}/{len(subtasks)}: {subtask}")

            # Simulate image capture for this subtask
            sim_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

            # Process subtask
            result = self.integrated_process(subtask, sim_image)
            task_results.append(result)

            print(f"  Subtask completed: {len(result['execution_log'])} actions executed")

        return {
            'task_description': task_description,
            'subtasks': subtasks,
            'results': task_results,
            'overall_success': all(len(r['execution_log']) > 0 for r in task_results)
        }

    def break_down_task(self, task_description):
        """
        Break down complex tasks into simpler subtasks
        """
        # Simple rule-based task breakdown
        if 'set table' in task_description.lower():
            return [
                "Pick up the plate",
                "Place the plate on the table",
                "Pick up the cup",
                "Place the cup next to the plate",
                "Pick up the fork",
                "Place the fork beside the plate"
            ]
        elif 'make coffee' in task_description.lower():
            return [
                "Locate the coffee mug",
                "Grasp the coffee mug",
                "Move to the coffee maker",
                "Place mug under coffee outlet",
                "Press coffee start button",
                "Wait for coffee to fill",
                "Remove filled mug"
            ]
        elif 'organize books' in task_description.lower():
            return [
                "Identify scattered books",
                "Pick up first book",
                "Move to bookshelf",
                "Place book on shelf",
                "Repeat for remaining books"
            ]
        else:
            # Single action if not a complex task
            return [task_description]

# Test the integrated VLA system
integrated_system = IntegratedVLASystem()

# Test complex tasks
complex_tasks = [
    "Set the dining table with plate, cup, and fork",
    "Organize the books on the shelf",
    "Make a cup of coffee"
]

print("\nIntegrated VLA System Tests:")
print("="*60)

for i, task in enumerate(complex_tasks):
    print(f"\nComplex Task {i+1}: {task}")

    result = integrated_system.execute_complex_task(task)

    print(f"Overall success: {result['overall_success']}")
    print(f"Completed {len(result['results'])} subtasks")
    print(f"Total actions executed: {sum(len(r['execution_log']) for r in result['results'])}")

print("\nVLA Mini-Experiments completed!")

# Summary statistics
print("\nSummary:")
print("- Experiment 1: Simple VLA system with object detection and action mapping")
print("- Experiment 2: Language-guided manipulation planning")
print("- Experiment 3: Vision-action coordination with servoing")
print("- Experiment 4: Integrated VLA system for complex tasks")
print("\nThe experiments demonstrate the progression from simple components to integrated systems.")
```

## Exercise Challenges

### Challenge 1: Robustness Testing
Modify the VLA system to handle:
- Noisy images with varying lighting conditions
- Ambiguous language commands
- Objects that are partially occluded

### Challenge 2: Performance Optimization
Improve the system's performance by:
- Reducing computation time for real-time operation
- Implementing caching for repeated computations
- Optimizing data structures for faster access

### Challenge 3: Error Recovery
Add error handling and recovery mechanisms:
- Detect when actions fail to execute
- Plan alternative strategies when primary plan fails
- Request human assistance when automated recovery fails

## Assessment Questions

1. How do the vision, language, and action components interact in your experiments?
2. What challenges arise when trying to integrate these components?
3. How does real-time performance factor into the design of VLA systems?
4. What safety considerations are important for VLA systems?

## Summary

These mini-experiments provide hands-on experience with VLA concepts in humanoid robotics. Through progressively complex implementations, you've explored how vision, language, and action can be coordinated to enable intelligent robot behavior. The integrated system demonstrates how individual components work together to accomplish complex tasks.
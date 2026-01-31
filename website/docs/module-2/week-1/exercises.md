---
title: "Week 1 Exercises: Gazebo Simulation Fundamentals"
description: "Practical exercises to reinforce understanding of Gazebo simulation fundamentals."
tags: [gazebo, simulation, fundamentals, exercises, robotics]
learning-objectives:
  - "Apply Gazebo simulation concepts to practical scenarios"
  - "Distinguish between simulation and real-world robotics"
  - "Analyze the implications of simulation for robotics development"
---

# Week 1 Exercises: Gazebo Simulation Fundamentals

## Learning Objectives

By completing these exercises, you will:
- Apply Gazebo simulation concepts to practical scenarios
- Gain hands-on experience with the Gazebo interface and controls
- Create and modify simple world files
- Understand the benefits and limitations of simulation-based development
- Develop critical thinking about the differences between simulation and real-world robotics

## Exercise 1: Gazebo Installation and Basic Setup

### Task
Install Gazebo on your system and verify the installation by launching the GUI and running a basic simulation.

### Setup
- Ubuntu 20.04 or 22.04
- ROS2 Humble Hawksbill
- Minimum 8GB RAM recommended

### Instructions
1. Install Gazebo Garden or Harmonic following the official installation guide
2. Verify installation by running `gz --version`
3. Launch Gazebo GUI with a simple world
4. Test basic interface controls (orbit, pan, zoom, focus)
5. Run a simulation with basic objects for 30 seconds

### Expected Output
- Gazebo version information displayed in terminal
- GUI successfully launched with default world
- Interface controls work correctly
- Simulation runs smoothly without errors

### Verification
- Check that Gazebo launches without errors
- Verify that basic simulation runs for the full 30 seconds
- Confirm that all interface controls function properly

### Troubleshooting
- If Gazebo fails to launch, check graphics driver compatibility
- If simulation runs slowly, reduce visual quality settings
- If interface controls don't work, ensure you're using the correct mouse buttons

---

## Exercise 2: Basic World Creation

### Task
Create a custom world file with a ground plane and multiple objects arranged in a specific configuration.

### Setup
- Text editor for SDF creation
- Gazebo to test the world file

### Instructions
1. Create an SDF world file named `custom_environment.sdf`
2. Include a ground plane with appropriate physics properties
3. Add at least 3 different objects (boxes, spheres, cylinders) with varying colors
4. Arrange objects in a rectangular pattern (2x2 formation)
5. Include proper lighting configuration
6. Test the world in Gazebo

### Expected Output
- World file with properly formatted SDF structure
- Objects arranged in specified configuration
- Physics simulation works correctly
- Lighting illuminates the scene appropriately

### Verification
- World loads without errors in Gazebo
- All objects are visible and properly positioned
- Objects interact with physics (e.g., boxes don't fall through ground)
- Lighting is visible and appropriate

### Troubleshooting
- If objects fall through the ground, check collision properties
- If world fails to load, verify XML syntax and SDF version
- If lighting appears incorrect, check light direction and intensity

---

## Exercise 3: Object Spawning and Manipulation

### Task
Use the command-line tools to spawn objects into a running simulation and manipulate their properties.

### Setup
- Gazebo running with a basic world
- Terminal access for command-line tools

### Instructions
1. Launch a basic Gazebo world (empty world or simple environment)
2. Spawn a box model at position (1, 0, 0.5) with default orientation
3. Spawn a sphere model at position (-1, 1, 0.5) with rotation of 45 degrees around Z-axis
4. Verify that both objects appear in the correct positions
5. Optionally, try changing properties of spawned objects during simulation

### Expected Output
- Box appears at coordinates (1, 0, 0.5)
- Sphere appears at coordinates (-1, 1, 0.5) with proper rotation
- Both objects respond to physics (e.g., sphere falls to ground)
- Objects don't collide with each other inappropriately

### Verification
- Use `gz model -l` to list all models in the simulation
- Visually confirm object positions match expected values
- Check that objects behave according to physics (falling, stability)
- Verify that spawned objects integrate properly with the simulation

### Troubleshooting
- If spawn command fails, check model name and path
- If objects don't appear, verify coordinates are within visible range
- If physics behave unexpectedly, check object properties and mass

---

## Exercise 4: Simulation Parameters Exploration

### Task
Experiment with different simulation parameters and observe their effects on simulation behavior.

### Setup
- Gazebo with a world containing multiple objects
- Ability to modify physics parameters

### Instructions
1. Create a world with several objects (at least 5-6 items)
2. Test the simulation with default parameters (record behavior)
3. Modify time step to 0.0005 (half the default) and observe changes
4. Modify real-time factor to 0.5 (slow motion) and 2.0 (fast motion)
5. Change gravity to different values (e.g., moon gravity: 0 0 -1.62)
6. Document differences in behavior between parameter sets

### Expected Output
- Different parameter values produce visibly different simulation behaviors
- Smaller time steps result in more accurate but slower simulation
- Real-time factor changes affect simulation speed relative to wall-clock time
- Different gravity values affect object motion appropriately

### Verification
- Record specific observations for each parameter set
- Confirm that physics behave as expected with different gravity
- Verify that simulation remains stable across parameter changes
- Document performance differences (frames per second, CPU usage)

### Troubleshooting
- If simulation becomes unstable with small time steps, check for numerical errors
- If performance degrades significantly, consider simplifying the world
- If objects behave unexpectedly, verify that parameters are applied correctly

---

## Exercise 5: Interface Navigation Mastery

### Task
Demonstrate proficiency with Gazebo's interface controls and features.

### Setup
- Gazebo with a moderately complex world
- Mouse with middle button and scroll wheel

### Instructions
1. Load a world with multiple objects (5+ items)
2. Navigate to focus on each object using double-click
3. Orbit around the scene to view from different angles
4. Pan the camera to different areas of the scene
5. Zoom in and out to different extents
6. Use the time panel to pause, step, and resume simulation
7. Explore the model list and select different objects
8. Document the most useful navigation techniques

### Expected Output
- Proficiency in camera navigation controls
- Understanding of time control features
- Ability to quickly focus on specific objects
- Familiarity with the interface panels and tools

### Verification
- Complete navigation tasks efficiently (under 5 minutes)
- Demonstrate all required navigation techniques
- Show understanding of how interface features enhance workflow
- Document personal preferences for navigation methods

### Troubleshooting
- If navigation feels sluggish, check graphics settings
- If interface elements are not visible, ensure proper Gazebo version
- If controls don't work as expected, verify mouse/keyboard setup

---

## Exercise 6: Custom World Documentation

### Task
Create comprehensive documentation for your custom world file, explaining each element and its purpose.

### Setup
- Your custom world file from Exercise 2
- Text editor for documentation

### Instructions
1. Create a detailed README file for your world
2. Document the purpose of each SDF element (physics, lights, models, etc.)
3. Explain the reasoning behind your design choices
4. Include troubleshooting tips for common issues
5. Add performance notes regarding the simulation
6. Provide suggestions for modifications and extensions

### Expected Output
- Well-structured documentation file
- Clear explanations of each SDF element
- Design rationale for the world configuration
- Helpful information for others using the world file

### Verification
- Someone else should be able to understand your world file from the documentation
- All major elements are explained clearly
- Troubleshooting information is accurate and helpful
- Performance notes are realistic and informative

### Troubleshooting
- If documentation is unclear, have someone else review it
- If elements are missing from documentation, add them
- If explanations are too technical, add simpler descriptions

---

## Troubleshooting Tips

- **Performance Issues**: Reduce visual quality, simplify collision meshes, decrease world complexity
- **Physics Instabilities**: Increase solver iterations, reduce time step, adjust contact parameters
- **Graphics Problems**: Update graphics drivers, check OpenGL compatibility, adjust rendering settings
- **Loading Failures**: Verify SDF syntax, check file paths, ensure proper XML structure

## Challenge Extension

For advanced students: Create a more complex simulation environment that includes:
- Multiple lighting sources with different properties
- A combination of static and dynamic objects
- Environmental effects (fog, shadows, reflections)
- Custom textures or materials
- Interactive elements that respond to simulation events

---

## Summary

These exercises provided hands-on experience with Gazebo simulation fundamentals, from basic installation and interface navigation to world creation and parameter exploration. Understanding these foundational concepts is crucial for effective use of Gazebo in robotics development.

## Next Steps

After completing these exercises, continue to [Week 2: Robot Model Integration](../week-2/) to learn how to import and configure robot models in Gazebo.
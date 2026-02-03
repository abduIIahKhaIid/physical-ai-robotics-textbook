---
title: "Lab 2: Perception Pipeline Implementation"
description: "Hands-on lab to implement a perception pipeline using Isaac ROS"
tags: [lab, perception, pipeline, isaac-ros, robotics, sensors, camera, lidar]
sidebar_position: 3
---

# Lab 2: Perception Pipeline Implementation

## Learning Objectives

After completing this lab, you will be able to:
- Implement a basic perception pipeline using Isaac ROS nodes
- Process camera data for object detection
- Process LiDAR data for obstacle detection
- Integrate camera and LiDAR data using sensor fusion
- Verify perception pipeline functionality through simulation

## Prerequisites

Before starting this lab, you must:
- Complete Week 1 lessons and Lab 1
- Complete Week 2 lessons on perception pipelines
- Have Isaac Sim and Isaac ROS properly installed and configured
- Have a working understanding of ROS2 concepts
- Successfully complete Lab 1: Isaac Sim Basics

## Equipment and Software Required

- NVIDIA Isaac Sim with robot scene loaded
- Isaac ROS packages installed and sourced
- Compatible GPU with CUDA support
- Robot with camera and LiDAR sensors configured
- Basic knowledge of ROS2 tools (ros2 run, ros2 launch, etc.)

## Estimated Time

This lab should take approximately 90-120 minutes to complete.

## Pre-Lab Setup

1. Ensure Isaac Sim is properly installed and working
2. Verify Isaac ROS packages are installed and sourced
3. Launch Isaac Sim and load a scene with obstacles
4. Verify camera and LiDAR sensors are properly configured on your robot
5. Test that sensor data is being published on appropriate ROS topics

## Lab Procedure

### Step 1: Verify Isaac ROS Installation

1. Open a terminal and source ROS2:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. Source Isaac ROS:
   ```bash
   source /opt/isaac_ros_ws/install/setup.bash
   ```

3. Verify Isaac ROS packages are available:
   ```bash
   ros2 pkg list | grep isaac_ros
   ```

4. You should see packages like `isaac_ros_image_proc`, `isaac_ros_pointcloud_utils`, etc.

### Step 2: Launch Isaac Sim Scene with Obstacles

1. In Isaac Sim, open a scene with obstacles (e.g., Simple Room with objects)
2. Ensure your robot has both camera and LiDAR sensors configured
3. Position the robot to face some objects in the scene
4. Start the simulation in Isaac Sim

### Step 3: Verify Sensor Data Publication

1. Open a new terminal and source ROS2 and Isaac ROS
2. Check available topics:
   ```bash
   ros2 topic list
   ```

3. Verify camera topic is active:
   ```bash
   ros2 topic echo /camera/color/image_raw --field header.stamp
   ```

4. Verify LiDAR topic is active:
   ```bash
   ros2 topic echo /lidar/points --field header.stamp
   ```

5. Press Ctrl+C to stop the echo commands after verifying data is flowing

### Step 4: Launch Isaac ROS Image Processing Pipeline

1. Create a launch file for the perception pipeline (save as `perception_pipeline.launch.py`):
   ```python
   from launch import LaunchDescription
   from launch.actions import DeclareLaunchArgument
   from launch.substitutions import LaunchConfiguration
   from launch_ros.actions import ComposableNodeContainer
   from launch_ros.descriptions import ComposableNode

   def generate_launch_description():
       # Launch Arguments
       use_sim_time = LaunchConfiguration('use_sim_time', default='True')

       perception_node_container = ComposableNodeContainer(
           name='perception_node_container',
           namespace='',
           package='rclcpp_components',
           executable='component_container_mt',
           composable_node_descriptions=[
               # Image Rectification
               ComposableNode(
                   package='isaac_ros_image_proc',
                   plugin='isaac_ros::ImageProcComponent',
                   name='image_proc',
                   parameters=[{
                       'use_sensor_data_qos': True,
                       'input_width': 640,
                       'input_height': 480,
                   }],
                   remappings=[
                       ('image_raw', '/camera/color/image_raw'),
                       ('camera_info', '/camera/color/camera_info'),
                       ('image_rect', '/camera/color/image_rect'),
                   ],
               ),
           ],
           output='screen',
       )

       return LaunchDescription([
           perception_node_container,
       ])
   ```

2. Launch the image processing pipeline:
   ```bash
   ros2 launch perception_pipeline.launch.py
   ```

3. In another terminal, verify the rectified image topic:
   ```bash
   ros2 topic echo /camera/color/image_rect --field header.stamp
   ```

### Step 5: Implement Basic Object Detection

1. Create a simple object detection node (`basic_object_detector.py`):
   ```python
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Image
   from cv_bridge import CvBridge
   import cv2
   import numpy as np

   class BasicObjectDetector(Node):
       def __init__(self):
           super().__init__('basic_object_detector')

           # Create subscription to rectified image
           self.subscription = self.create_subscription(
               Image,
               '/camera/color/image_rect',
               self.image_callback,
               10)

           # Create publisher for annotated image
           self.publisher = self.create_publisher(
               Image,
               '/object_detection_result',
               10)

           # Initialize CV bridge
           self.cv_bridge = CvBridge()

           self.get_logger().info('Basic Object Detector initialized')

       def image_callback(self, msg):
           """Process image and detect objects"""
           try:
               # Convert ROS image to OpenCV
               cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")

               # Convert to grayscale for edge detection
               gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

               # Apply Gaussian blur
               blurred = cv2.GaussianBlur(gray, (5, 5), 0)

               # Apply Canny edge detection
               edges = cv2.Canny(blurred, 50, 150)

               # Find contours
               contours, _ = cv2.findContours(
                   edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

               # Draw bounding boxes around detected objects
               annotated_image = cv_image.copy()
               for contour in contours:
                   # Filter small contours
                   if cv2.contourArea(contour) > 200:  # Minimum area threshold
                       x, y, w, h = cv2.boundingRect(contour)

                       # Draw rectangle
                       cv2.rectangle(annotated_image, (x, y), (x+w, y+h), (0, 255, 0), 2)

                       # Add label
                       cv2.putText(annotated_image, 'Object', (x, y-10),
                                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

               # Convert back to ROS image
               result_msg = self.cv_bridge.cv2_to_imgmsg(annotated_image, "bgr8")
               result_msg.header = msg.header

               # Publish result
               self.publisher.publish(result_msg)

           except Exception as e:
               self.get_logger().error(f'Error in image callback: {e}')

   def main(args=None):
       rclpy.init(args=args)
       detector = BasicObjectDetector()

       try:
           rclpy.spin(detector)
       except KeyboardInterrupt:
           pass
       finally:
           detector.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. Make the script executable:
   ```bash
   chmod +x basic_object_detector.py
   ```

3. Run the object detection node:
   ```bash
   python3 basic_object_detector.py
   ```

### Step 6: Process LiDAR Data for Obstacle Detection

1. Create a LiDAR processing node (`lidar_processor.py`):
   ```python
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import PointCloud2
   from sensor_msgs_py import point_cloud2
   from std_msgs.msg import String
   import numpy as np
   from sklearn.cluster import DBSCAN

   class LidarProcessor(Node):
       def __init__(self):
           super().__init__('lidar_processor')

           # Subscription to LiDAR data
           self.subscription = self.create_subscription(
               PointCloud2,
               '/lidar/points',
               self.pointcloud_callback,
               10)

           # Publisher for obstacle information
           self.obstacle_publisher = self.create_publisher(
               String,
               '/obstacle_detected',
               10)

           self.get_logger().info('LiDAR Processor initialized')

       def pointcloud_callback(self, msg):
           """Process LiDAR point cloud data"""
           try:
               # Extract points from PointCloud2 message
               points_list = []
               for point in point_cloud2.read_points(
                   msg,
                   field_names=("x", "y", "z"),
                   skip_nans=True):
                   points_list.append([point[0], point[1], point[2]])

               if not points_list:
                   return

               points = np.array(points_list)

               # Filter ground plane (remove points with z < -0.5)
               filtered_points = points[points[:, 2] > -0.5]

               if len(filtered_points) == 0:
                   return

               # Extract x,y coordinates for clustering
               xy_coords = filtered_points[:, :2]

               # Perform DBSCAN clustering for obstacle detection
               clustering = DBSCAN(eps=0.7, min_samples=15)
               cluster_labels = clustering.fit_predict(xy_coords)

               # Count unique clusters (obstacles)
               unique_clusters = set(cluster_labels)
               n_clusters = len(unique_clusters) - (1 if -1 in unique_clusters else 0)  # Exclude noise points

               # Publish obstacle count
               obstacle_msg = String()
               obstacle_msg.data = f'Detected {n_clusters} obstacles'
               self.obstacle_publisher.publish(obstacle_msg)

               self.get_logger().info(f'Detected {n_clusters} obstacles')

           except Exception as e:
               self.get_logger().error(f'Error processing point cloud: {e}')

   def main(args=None):
       rclpy.init(args=args)
       processor = LidarProcessor()

       try:
           rclpy.spin(processor)
       except KeyboardInterrupt:
           pass
       finally:
           processor.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. Make the script executable and run it:
   ```bash
   chmod +x lidar_processor.py
   python3 lidar_processor.py
   ```

### Step 7: Implement Sensor Fusion

1. Create a fusion node (`sensor_fusion_node.py`):
   ```python
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Image, PointCloud2
   from std_msgs.msg import String
   from cv_bridge import CvBridge
   import numpy as np
   from threading import Lock

   class SensorFusionNode(Node):
       def __init__(self):
           super().__init__('sensor_fusion_node')

           # Subscriptions for both sensors
           self.image_sub = self.create_subscription(
               Image,
               '/camera/color/image_rect',
               self.image_callback,
               10)

           self.lidar_sub = self.create_subscription(
               PointCloud2,
               '/lidar/points',
               self.lidar_callback,
               10)

           # Publisher for fused results
           self.fusion_publisher = self.create_publisher(
               String,
               '/fusion_result',
               10)

           # Storage for synchronized data
           self.latest_image = None
           self.latest_lidar = None
           self.data_lock = Lock()

           # CV bridge
           self.cv_bridge = CvBridge()

           self.get_logger().info('Sensor Fusion Node initialized')

       def image_callback(self, msg):
           """Store latest image"""
           with self.data_lock:
               self.latest_image = msg

       def lidar_callback(self, msg):
           """Process LiDAR and attempt fusion with image"""
           with self.data_lock:
               if self.latest_image is not None:
                   self.perform_fusion(self.latest_image, msg)
                   # Clear image after fusion attempt
                   self.latest_image = None

       def perform_fusion(self, image_msg, pointcloud_msg):
           """Perform basic fusion between camera and LiDAR"""
           try:
               # In this simple example, we'll just combine the information
               # Convert image to get dimensions
               cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
               img_height, img_width = cv_image.shape[:2]

               # Process LiDAR points
               points_list = []
               for point in point_cloud2.read_points(
                   pointcloud_msg,
                   field_names=("x", "y", "z"),
                   skip_nans=True):
                   points_list.append([point[0], point[1], point[2]])

               if points_list:
                   points = np.array(points_list)
                   # Simple fusion: count objects from camera and obstacles from LiDAR
                   # In a real system, you'd project LiDAR points to image coordinates

                   fusion_result = String()
                   fusion_result.data = f"Fusion: Image objects detected, {len(points)} LiDAR points processed"
                   self.fusion_publisher.publish(fusion_result)

                   self.get_logger().info(f'Fusion result published: {fusion_result.data}')

           except Exception as e:
               self.get_logger().error(f'Error in fusion: {e}')

   def main(args=None):
       rclpy.init(args=args)
       fusion_node = SensorFusionNode()

       try:
           rclpy.spin(fusion_node)
       except KeyboardInterrupt:
           pass
       finally:
           fusion_node.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. Run the fusion node:
   ```bash
   python3 sensor_fusion_node.py
   ```

### Step 8: Monitor All Nodes Together

1. In separate terminals, run all nodes:
   - Image processing pipeline (launch file)
   - Object detection node
   - LiDAR processing node
   - Sensor fusion node

2. Monitor the results:
   ```bash
   # Monitor object detection results
   ros2 topic echo /object_detection_result

   # Monitor obstacle detection
   ros2 topic echo /obstacle_detected

   # Monitor fusion results
   ros2 topic echo /fusion_result
   ```

### Step 9: Test with Different Scenarios

1. In Isaac Sim, move the robot to different positions
2. Change the robot's orientation to face different objects
3. Add/remove objects in the scene to test detection capabilities
4. Observe how the perception pipeline responds to different scenarios

### Step 10: Analyze Performance

1. Monitor the processing rate of each node
2. Check for any dropped messages
3. Observe the responsiveness of the perception system
4. Note any delays or performance bottlenecks

## Expected Results

During this lab, you should observe:
- Camera images being processed and objects being detected with bounding boxes
- LiDAR data being processed to identify obstacles in the environment
- Sensor fusion combining information from both modalities
- All nodes running stably with minimal message drops
- Perception system responding appropriately to changes in the scene

## Verification Steps

To verify your lab was successful:

1. Image processing pipeline successfully receives and processes camera data
2. Object detection node identifies objects in the scene with bounding boxes
3. LiDAR processing node detects obstacles in the environment
4. Sensor fusion node combines information from both sensors
5. All nodes communicate properly through ROS topics
6. Perception system responds to changes in the simulation environment

## Troubleshooting

Common issues and solutions:

- **No image data**: Verify Isaac Sim is publishing camera data and topic names match
- **LiDAR not publishing**: Check LiDAR sensor configuration in Isaac Sim
- **Nodes not connecting**: Verify all nodes are using the same ROS domain ID
- **Performance issues**: Reduce image resolution or processing rate to match your hardware
- **Import errors**: Ensure Isaac ROS packages are properly sourced in your environment

## Lab Questions

Answer these questions to confirm your understanding:

1. How does the camera processing pipeline enhance raw image data?
2. What is the purpose of clustering in LiDAR processing?
3. How does sensor fusion improve perception compared to using single sensors?
4. What are the main computational bottlenecks in perception processing?

## Conclusion

In this lab, you successfully implemented a complete perception pipeline using Isaac ROS. You processed camera data for object detection, processed LiDAR data for obstacle detection, and implemented basic sensor fusion. This forms the foundation for more advanced perception systems that you'll encounter in later modules.

## References

- [Isaac ROS Perception Packages](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_perception/index.html)
- [ROS2 Image Transport Documentation](https://docs.ros.org/en/humble/p/r/image_transport/)
- [Point Cloud Library Documentation](https://pcl.readthedocs.io/projects/tutorials/en/latest/)

:::caution
Ensure all nodes are properly shut down before closing Isaac Sim to avoid resource conflicts.
:::

:::tip
Save your launch files and scripts for future reference and potential extension.
:::
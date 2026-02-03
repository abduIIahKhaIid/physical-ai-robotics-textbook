# Isaac ROS Navigation Pipeline Example
# This is a basic example of a navigation pipeline using Isaac ROS

import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import math

class NavigationPipeline:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('navigation_pipeline')

        # Publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Robot state
        self.current_pose = None
        self.current_twist = None
        self.laser_data = None

        # Navigation parameters
        self.linear_speed = 0.2
        self.angular_speed = 0.3
        self.safe_distance = 0.5

        # Goal parameters
        self.target_x = 0.0
        self.target_y = 0.0
        self.reached_goal = False

        print("Navigation pipeline initialized")

    def odom_callback(self, msg):
        """Update robot pose from odometry"""
        self.current_pose = msg.pose.pose
        self.current_twist = msg.twist.twist

    def scan_callback(self, msg):
        """Update laser scan data"""
        self.laser_data = msg

    def get_robot_position(self):
        """Get current robot position"""
        if self.current_pose:
            x = self.current_pose.position.x
            y = self.current_pose.position.y
            z = self.current_pose.position.z
            return x, y, z
        return None, None, None

    def calculate_distance(self, x1, y1, x2, y2):
        """Calculate Euclidean distance between two points"""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi] range"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def get_yaw_from_quaternion(self, quaternion):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def check_obstacles(self):
        """Check for obstacles in front of robot"""
        if not self.laser_data:
            return False

        # Check the front 30-degree sector
        angle_min = self.laser_data.angle_min
        angle_increment = self.laser_data.angle_increment
        ranges = self.laser_data.ranges

        # Front sector is roughly 15 degrees left and right of center
        center_idx = len(ranges) // 2
        sector_width = int(15 * math.pi / 180 / angle_increment)  # 15 degrees in indices

        for i in range(center_idx - sector_width, center_idx + sector_width):
            if 0 <= i < len(ranges):
                if ranges[i] < self.safe_distance and not math.isnan(ranges[i]):
                    return True

        return False

    def move_towards_goal(self):
        """Move robot towards the goal"""
        if not self.current_pose:
            return

        # Get current position
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)

        # Calculate distance to goal
        dist_to_goal = self.calculate_distance(current_x, current_y, self.target_x, self.target_y)

        # Check if reached goal
        if dist_to_goal < 0.2:  # 20cm tolerance
            self.reached_goal = True
            self.stop_robot()
            return

        # Check for obstacles
        if self.check_obstacles():
            print("Obstacle detected, stopping robot")
            self.stop_robot()
            return

        # Calculate angle to goal
        angle_to_goal = math.atan2(self.target_y - current_y, self.target_x - current_x)
        angle_diff = self.normalize_angle(angle_to_goal - current_yaw)

        # Create twist message
        twist = Twist()

        # Set angular velocity based on angle difference
        if abs(angle_diff) > 0.1:  # 0.1 radian threshold
            twist.angular.z = max(-self.angular_speed, min(self.angular_speed, angle_diff))
        else:
            # Move forward if roughly facing the goal
            twist.linear.x = self.linear_speed

        # Publish the command
        self.cmd_vel_pub.publish(twist)

    def set_goal(self, x, y):
        """Set navigation goal"""
        self.target_x = x
        self.target_y = y
        self.reached_goal = False

        # Publish goal for visualization
        goal_msg = PoseStamped()
        goal_msg.header = Header()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position = Point(x, y, 0.0)
        goal_msg.pose.orientation.w = 1.0

        self.goal_pub.publish(goal_msg)

    def stop_robot(self):
        """Stop the robot"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def run(self):
        """Run the navigation pipeline"""
        rate = rospy.Rate(10)  # 10 Hz

        # Set a sample goal
        self.set_goal(1.0, 1.0)

        while not rospy.is_shutdown():
            if not self.reached_goal:
                self.move_towards_goal()

            rate.sleep()

        # Stop robot on shutdown
        self.stop_robot()

if __name__ == '__main__':
    try:
        nav_pipeline = NavigationPipeline()
        nav_pipeline.run()
    except rospy.ROSInterruptException:
        print("Navigation pipeline interrupted")
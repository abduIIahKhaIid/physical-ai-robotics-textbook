# Isaac ROS Perception Pipeline Example
# This is a basic example of a perception pipeline using Isaac ROS

import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy

class PerceptionPipeline:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('perception_pipeline')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Subscribe to camera topic
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)

        # Publisher for processed image
        self.image_pub = rospy.Publisher("/perception/output_image", Image, queue_size=10)

        # Object detection parameters
        self.confidence_threshold = 0.5

        print("Perception pipeline initialized")

    def image_callback(self, data):
        """Process incoming image data"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # Process the image for object detection
            processed_image = self.detect_objects(cv_image)

            # Convert back to ROS image format
            ros_image = self.bridge.cv2_to_imgmsg(processed_image, "bgr8")
            ros_image.header.stamp = rospy.Time.now()
            ros_image.header.frame_id = data.header.frame_id

            # Publish the processed image
            self.image_pub.publish(ros_image)

        except Exception as e:
            print(f"Error processing image: {str(e)}")

    def detect_objects(self, image):
        """Detect objects in the image"""
        # This is a simplified example - in practice, you'd use a neural network
        # or other computer vision techniques

        # Convert to grayscale for edge detection
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Apply Canny edge detection
        edges = cv2.Canny(blurred, 50, 150)

        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw bounding boxes around detected objects
        output_image = image.copy()
        for contour in contours:
            # Filter small contours
            if cv2.contourArea(contour) > 100:
                x, y, w, h = cv2.boundingRect(contour)

                # Draw rectangle
                cv2.rectangle(output_image, (x, y), (x+w, y+h), (0, 255, 0), 2)

                # Add label
                cv2.putText(output_image, 'Object', (x, y-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        return output_image

    def run(self):
        """Run the perception pipeline"""
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down perception pipeline")

if __name__ == '__main__':
    pipeline = PerceptionPipeline()
    pipeline.run()
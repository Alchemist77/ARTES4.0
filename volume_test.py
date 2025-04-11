import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import time

class VolumeEstimator:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('volume_estimator', anonymous=True)

        # Create CvBridge object
        self.bridge = CvBridge()

        # Intrinsic camera parameters
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # Subscribers
        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback)
        rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback)
        rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)

        # Depth and color image placeholders
        self.depth_image = None
        self.color_image = None

        # Baseline depth image
        self.baseline_depth = None
        self.start_time = time.time()
        self.baseline_captured = False

    def camera_info_callback(self, msg):
        # Retrieve intrinsic camera parameters from CameraInfo message
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]

    def depth_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Capture baseline depth for the first 5 seconds
        if not self.baseline_captured and (time.time() - self.start_time) > 5:
            self.baseline_depth = self.depth_image.copy()
            self.baseline_captured = True
            rospy.loginfo("Baseline depth image captured.")

    def color_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def calculate_volume(self):
        if self.depth_image is None or self.color_image is None or self.fx is None or self.fy is None:
            rospy.logwarn("Waiting for depth image, color image, and camera info...")
            return

        if self.baseline_depth is None:
            rospy.logwarn("Waiting for baseline depth image...")
            return

        # Calculate difference from baseline to detect the object
        depth_diff = self.depth_image - self.baseline_depth
        depth_diff[depth_diff < 50] = 0  # Set a threshold to ignore small changes
        depth_diff[depth_diff > 5000] = 0  # Remove extreme values (outliers)

        # Convert depth difference to binary image
        binary_diff = np.where(depth_diff > 0, 255, 0).astype(np.uint8)

        # Find contours of the detected regions
        contours, _ = cv2.findContours(binary_diff, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            rospy.logwarn("No significant object detected in depth image.")
            return

        # Find the largest contour (assuming it corresponds to the object)
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        object_roi = depth_diff[y:y+h, x:x+w]

        # Calculate average depth in the object ROI
        valid_depths = object_roi[object_roi > 0]
        if valid_depths.size == 0:
            rospy.logwarn("No valid depth data found in detected object region.")
            return

        average_depth = np.mean(valid_depths)

        # Estimate width, height, and depth of the object in real-world units (in cm)
        W = (w * average_depth) / self.fx / 10.0  # Width in cm
        H = (h * average_depth) / self.fy / 10.0  # Height in cm
        D = average_depth / 10.0  # Depth in cm

        # Calculate volume assuming a box shape (in cubic cm)
        volume = W * H * D
        rospy.loginfo(f"Estimated Volume: {volume:.2f} cubic centimeters")

        # Visualize the volume area on the color image
        self.visualize_volume_on_color_image(volume, x, y, w, h)

    def visualize_volume_on_color_image(self, volume, x, y, w, h):
        if self.color_image is None:
            return

        # Create a copy of the color image to draw on
        image_with_volume = self.color_image.copy()

        # Draw bounding box around the detected object
        cv2.rectangle(image_with_volume, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Add volume text to the image
        cv2.putText(image_with_volume, f"Estimated Volume: {volume:.2f} cm^3", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Display the color image with the volume information
        cv2.imshow("Color Image with Volume Estimation", image_with_volume)
        cv2.waitKey(1)

    def run(self):
        # Set the rate to calculate volume periodically
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            self.calculate_volume()
            rate.sleep()

if __name__ == '__main__':
    try:
        volume_estimator = VolumeEstimator()
        volume_estimator.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()

"#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import socket
import threading
import sys
sys.path.append('/path/to/your/module')
import csv_helper as csvHelp

HOST = '127.0.0.1'  # IP del tuo PC locale
PORT = 65432            # La stessa porta del server

class ColorBoxRecognizer:
    def __init__(self):
        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.camera_info = None
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.color_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
        self.camera_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)
        ###########------------COMMUNICATION SERVER -------------------------###################
        self.host = HOST
        self.port = PORT
        self.current_volume_ = 0 ## SWITCH WITH THE ACTUAL VALUE TO CHANGE
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.runInBackground()
        self.table_depth_reference = 0.716  # Set a fixed table depth reference in meters

    def startServer(self):
        """Avvia il server socket su un thread separato."""
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen()
        print(f"Server in ascolto su {self.host}:{self.port}...")
        
        while True:
            conn, addr = self.server_socket.accept()
            print(f"Connessione da {addr}")
            client_thread = threading.Thread(target=self.handleClient, args=(conn,))
            client_thread.start()

    def handleClient(self, conn):
        """Gestisce ogni client connesso al server."""
        with conn:
            data = conn.recv(1024)
            if data == b"SAVE DATA":
                # Save data to file
                try:
                    csvHelp.saveToCsv(self.current_volume_)
                    response = "OK".encode('utf-8')
                except:
                    response = "NOK".encode('utf-8')
                conn.sendall(response)

    def runInBackground(self):
        """Avvia il server socket su un thread separato."""
        server_thread = threading.Thread(target=self.startServer)
        server_thread.daemon = True  # Assicura che il thread si chiuda con il programma principale
        server_thread.start()
        print("Server avviato in background.")

    def stopServer(self):
        self.server_socket.close()
        
    ################--------------------------------------------------------------###################

    def color_callback(self, data):
        try:
            # Convert the ROS Image message to OpenCV format
            self.color_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        # Convert the BGR image to HSV
        hsv_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2HSV)

        # Define HSV ranges for blue, green, yellow, and red colors
        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([140, 255, 255])

        lower_green = np.array([35, 100, 100])
        upper_green = np.array([85, 255, 255])

        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])

        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        # Threshold the HSV image to get only blue, green, yellow, and red colors
        mask_blue = cv2.inRange(hsv_image, lower_blue, upper_blue)
        mask_green = cv2.inRange(hsv_image, lower_green, upper_green)
        mask_yellow = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
        mask_red1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        # Combine masks
        mask = cv2.bitwise_or(mask_blue, mask_green)
        mask = cv2.bitwise_or(mask, mask_yellow)
        mask = cv2.bitwise_or(mask, mask_red)

        # Perform morphological operations to clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Draw bounding boxes around detected objects
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            color = (0, 255, 0)  # Default to green
            label = "green"

            # Determine the color of the bounding box
            mask_subregion = mask[y:y+h, x:x+w]
            blue_mean = np.mean(mask_blue[y:y+h, x:x+w])
            green_mean = np.mean(mask_green[y:y+h, x:x+w])
            yellow_mean = np.mean(mask_yellow[y:y+h, x:x+w])
            red_mean = np.mean(mask_red[y:y+h, x:x+w])

            if blue_mean > green_mean and blue_mean > yellow_mean and blue_mean > red_mean:
                color = (255, 0, 0)  # Blue
                label = "blue"
            elif yellow_mean > blue_mean and yellow_mean > green_mean and yellow_mean > red_mean:
                color = (0, 255, 255)  # Yellow
                label = "yellow"
            elif red_mean > blue_mean and red_mean > green_mean and red_mean > yellow_mean:
                color = (0, 0, 255)  # Red
                label = "red"
            else:
                color = (0, 255, 0)  # Green (default)
                label = "green"

            self.color_image = cv2.rectangle(self.color_image, (x, y), (x + w, y + h), color, 2)
            # Put the label at the bottom right corner of the bounding box
            label_position = (x + w - 10, y + h - 10)
            cv2.putText(self.color_image, label, label_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2, cv2.LINE_AA)

    def depth_callback(self, data):
        try:
            # Convert the ROS Image message to OpenCV format
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError as e:
            print(e)

    def camera_info_callback(self, data):
        self.camera_info = data

    def process_images(self):
        if self.color_image is None or self.depth_image is None or self.camera_info is None:
            return

        # Extract camera intrinsic parameters
        fx = self.camera_info.K[0]  # Focal length in x direction
        fy = self.camera_info.K[4]  # Focal length in y direction
        cx = self.camera_info.K[2]  # Principal point x
        cy = self.camera_info.K[5]  # Principal point y

        # Define a 200x200 pixel boundary at the center of the image
        height, width = self.depth_image.shape
        center_x, center_y = width // 2, height // 2
        boundary_size = 200
        top_left_x = center_x - boundary_size
        top_left_y = center_y - boundary_size
        bottom_right_x = center_x + boundary_size
        bottom_right_y = center_y + boundary_size

        # Draw the boundary on the color image
        cv2.rectangle(self.color_image, (top_left_x, top_left_y), (bottom_right_x, bottom_right_y), (0, 255, 0), 2)

        # Extract depth information from the center boundary
        depth_roi = self.depth_image[top_left_y:bottom_right_y, top_left_x:bottom_right_x]
        valid_depths = depth_roi[depth_roi > 0]
        median_depth = np.median(valid_depths) / 1000.0 if valid_depths.size > 0 else 0.0  # Convert to meters

        # Compare current median depth with fixed table depth reference
        if abs(self.table_depth_reference - median_depth) < 0.005:  # If the difference is less than 10 mm
            avg_object_depth = 0.0  # No object detected
        else:
            avg_object_depth = median_depth
        print(f"Table Depth Reference: {self.table_depth_reference:.3f} m")
        print(f"Current Median Depth: {median_depth:.3f} m")
        print(f"Average Object Depth: {avg_object_depth:.3f} m")

        # If there are no objects above the table, set volume to zero
        if avg_object_depth == 0.0 or avg_object_depth >= self.table_depth_reference:
            volume = 0.0
            width_meters = 0.0
            height_meters = 0.0
            depth_meters = 0.0
        else:
            # Calculate volume using the average object depth
            width_meters = (bottom_right_x - top_left_x) * avg_object_depth / fx
            height_meters = (bottom_right_y - top_left_y) * avg_object_depth / fy
            depth_meters = self.table_depth_reference - avg_object_depth  # Object height above the table
            volume = width_meters * height_meters * depth_meters

        # Print intermediate values for debugging
        print(f"Width in Meters: {width_meters:.3f} m")
        print(f"Height in Meters: {height_meters:.3f} m")
        print(f"Depth (Object Height) in Meters: {depth_meters:.3f} m")
        print(f"Estimated Volume of Object Above Table: {volume:.5f} m^3")

        # Display the center depth value and volume on the color image
        center_depth_text = f"Table Depth Reference: {self.table_depth_reference:.3f} m"
        volume_text = f"Volume: {volume:.5f} m^3"
        text_position_depth = (top_left_x, top_left_y - 10)
        text_position_volume = (top_left_x, bottom_right_y + 20)
        cv2.putText(self.color_image, center_depth_text, text_position_depth, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(self.color_image, volume_text, text_position_volume, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)

        # Display the resulting frame
        cv_image_resized = cv2.resize(self.color_image, (640, 480))
        cv2.imshow('Center Depth and Volume Visualization', cv_image_resized)
        cv2.waitKey(3)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.process_images()
            rate.sleep()

def main():
    rospy.init_node('color_box_recognizer', anonymous=True)
    recognizer = ColorBoxRecognizer()
    try:
        recognizer.run()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
    recognizer.stopServer()

if __name__ == '__main__':
    main()
" 
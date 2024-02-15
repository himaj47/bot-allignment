#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class BallAlignmentNode(Node):
    def __init__(self):
        super().__init__('ball_alignment_node')

        self.last_y = 0
        self.largest_contour_area = 0

        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.detect_red_and_blue_balls_from_webcam()

    def publish_angular_velocity(self, angular_velocity):
        twist_msg = Twist()
        twist_msg.angular.z = float(angular_velocity)
        self.publisher_cmd_vel.publish(twist_msg)

    def publish_linear_velocity(self, linear_velocity):
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        self.publisher_cmd_vel.publish(twist_msg)

    def detect_balls(self,frame, contours, min_contour_area,color):
        # Calculate the centroids of valid contours
        centroids = []
        for cnt in contours:
            # Approximate the contour to check if it's a circle
            epsilon = 0.08 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)

            # Check if the contour has a small number of vertices (approximated as a circle)
            if len(approx) <= 6:
                # Check aspect ratio to filter out false positives
                x, y, w, h = cv2.boundingRect(cnt)
                aspect_ratio = float(w) / h
                if 0.95 <= aspect_ratio <= 1.05:
                    # Check contour area to filter out small contours
                    if cv2.contourArea(cnt) > min_contour_area:
                        M = cv2.moments(cnt)
                        if M["m00"] != 0:
                            cX = int(M["m10"] / M["m00"])
                            cY = int(M["m01"] / M["m00"])
                            centroids.append((cX, cY))

                            # Draw the contours and circles based on centroids
                            radius = 10  # You can adjust the radius as needed
                            cv2.circle(frame, (int(cX), int(cY)), radius, (0,255,0), 2)
                            print(cv2.contourArea(cnt))

        return centroids

    def detect_red_and_blue_balls_from_webcam(self):
        # Open the webcam
        cap = cv2.VideoCapture(0)  # Use 0 for the default camera, or change the index if you have multiple cameras

        while True:
            # Read a frame from the webcam
            ret, frame = cap.read()
            if not ret:
                break

            # Convert the frame from BGR to HSV color space
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Define the lower and upper bounds for the red color in HSV
            lower_red = np.array([0, 144, 133])
            upper_red = np.array([10, 255, 255])

            # Define the lower and upper bounds for the blue color in HSV
            lower_blue = np.array([90, 120, 100])
            upper_blue = np.array([120, 255, 255])

            # Create masks using the inRange function
            red_mask = cv2.inRange(hsv, lower_red, upper_red)
            blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

            # Find contours in the masks
            red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Filter out small contours , also Used for detecting balls at far distance 
            min_contour_area = 10

            # Detect red balls
            red_centroids = self.detect_balls(frame,  red_contours, min_contour_area, color=(0, 0, 255))

            # Detect blue balls
            blue_centroids = self.detect_balls(frame, blue_contours, min_contour_area, color=(255, 0, 0))

            # Print the centers of red balls
            print("Red Ball Centers:", red_centroids)

            # Print the centers of blue balls
            print("Blue Ball Centers:", blue_centroids)

            # Display the result
            cv2.imshow("Result", frame)

            # Break the loop when 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            self.yaw_align(blue_centroids)

        # Release the webcam and close the window
        cap.release()
        cv2.destroyAllWindows()


    def yaw_align(self, centroids):
        if centroids:
            x = centroids[0][0]  # Assuming the first centroid corresponds to the blue ball
            x_center = x-320

            if x < 310:
                angular_velocity = x_center  # Adjust the angular velocity as needed
            elif x > 330:
                angular_velocity = x_center  # Adjust the angular velocity as needed
            else:
                angular_velocity = x_center  # Stop angular motion

            # Publish the angular velocity
            # if self.last_y != x:  # Avoid unnecessary publishing
            #     self.publish_angular_velocity(angular_velocity)
            #     self.last_y = x

            self.publish_angular_velocity(angular_velocity)

    # def main(self):
    #     rclpy.spin_once(self)

    #     # After breaking from the webcam loop, start publishing linear velocity until largest_contour_area > 60000
    #     if self.largest_contour_area <= 60000:
    #         self.publish_linear_velocity(2.0)  # Set your desired linear velocity here


    

def main(args=None):
    rclpy.init(args=args)
    node = BallAlignmentNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()

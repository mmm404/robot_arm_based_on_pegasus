"""
Find the 3D pose of a human hand using computer vision.

Using cv_bridge, take messages from the Kinect2 camera and convert them
into OpenCV images for mediapipe to use to figure out where a user's hand
is located.

SUBSCRIBERS:
  + /kinect2/hd/image_color_rect (Image) - RGB image data from the Kinect2
  camera.
  + /kinect2/hd/image_depth_rect (Image) - Depth image from the Kinect2 camera.
PUBLISHERS:
  + /cv_image (Image) - Annotated image with the 3D location of the hand's
  pose.
  + /waypoint (PoseStamped) - The 3D location of the hand's pose.
  + /right_gesture (String) - The gesture that the right hand is making.
"""
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from hand_interfaces.msg import FingerData

from cv_bridge import CvBridge, CvBridgeError
from .mediapipehelper import MediaPipeRos as mps

import mediapipe as mp
import numpy as np
import cv2 as cv


class HandCV(Node):
    def __init__(self):
        super().__init__("handcv")

        # initialize CvBridge object
        self.bridge = CvBridge()

        # initialize MediaPipe Object
        self.mps = mps()

        # create callback groups
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        self.waypoint_callback_group = MutuallyExclusiveCallbackGroup()

        # create timer
        self.declare_parameter('update_rate', 30.0)
        update_rate = self.get_parameter('update_rate').value
        self.timer = self.create_timer(
            1/update_rate, self.timer_callback, callback_group=self.timer_callback_group)

        # create subscribers
        self.color_image_raw_sub = self.create_subscription(
            Image, '/kinect2/hd/image_color_rect', self.color_image_raw_callback, 10)

        self.depth_image_raw_sub = self.create_subscription(
            Image, '/kinect2/hd/image_depth_rect', self.depth_image_raw_callback, 10)

        # create publishers
        self.cv_image_pub = self.create_publisher(Image, 'cv_image', 10)

        self.waypoint_pub = self.create_publisher(
                PoseStamped, 'waypoint', 10)

        self.right_gesture_pub = self.create_publisher(
                String, 'right_gesture', 10)

        # intialize other variables
        self.color_image = None
        self.depth_image = None
        self.waypoint = PoseStamped() 
        self.waypoint.header.frame_id = 'kinect2_rgb_optical_frame'  # or 'camera_link' depending on your setup
        self.waypoint.pose.orientation.x = 1.0
        self.waypoint.pose.orientation.w = 0.0
        self.image_width = 0
        self.image_height = 0
        self.centroid = np.array([0.0, 0.0, 0.0])

    def depth_image_raw_callback(self, msg):
        """Capture depth images and convert them to OpenCV images."""
        try:
            # handle either 32FC1 (meters) or 16UC1 (millimeters) depth encodings
            if msg.encoding == "32FC1":
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            else:
                depth16 = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
                depth = depth16.astype(np.float32) / 1000.0 #mm -> meters
            # keep your horizontal flip if desired
            self.depth_image = cv.flip(depth, 1)
        except CvBridgeError as e:
            self.get_logger().error(str(e))

    def color_image_raw_callback(self, msg):
        """Cpature color images and convert them to OpenCV images."""
        self.color_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="bgr8")  # Kinect2 usually uses bgr8
        self.color_image = cv.flip(self.color_image, 1)
        self.image_width = msg.width
        self.image_height = msg.height

    def process_depth_image(self, annotated_image=None, detection_result=None):
        """
        Process the depth image to find the 3D location of the hand's pose.

        Args:
        ----
        annotated_image (np.array): The annotated image with the 3D location of
        the hand's pose.
        detection_result (mediapipe): The result of the hand landmark detection.

        Returns:
        -------
        cv_image (Image): The annotated image with the 3D location of the hand's
        pose.
        right_gesture (String): The gesture that the right hand is making.

        """
        if self.depth_image is None or annotated_image is None:
            self.get_logger().warn("Depth image or annotated image not yet received")
            if annotated_image is not None:
                return self.bridge.cv2_to_imgmsg(annotated_image, encoding="rgb8"), "None"
            return None, "None"

        # Validate depth image dimensions
        if self.depth_image.shape[0] == 0 or self.depth_image.shape[1] == 0:
            self.get_logger().warn("Invalid depth image dimensions")
            return self.bridge.cv2_to_imgmsg(annotated_image, encoding="rgb8"), "None"

        right_gesture = "None"
        right_index = None
        left_gesture = "None"
        left_index = None
        if detection_result.gestures and detection_result.handedness:
            if len(detection_result.handedness) == 2:
                if detection_result.handedness[0][0].category_name == "Left":
                    left_gesture = detection_result.gestures[0][0].category_name
                    right_gesture = detection_result.gestures[1][0].category_name
                    left_index = 0
                    right_index = 1
                elif detection_result.handedness[0][0].category_name == "Right":
                    right_gesture = detection_result.gestures[0][0].category_name
                    left_gesture = detection_result.gestures[1][0].category_name
                    left_index = 1
                    right_index = 0
            elif len(detection_result.handedness) == 1:
                if detection_result.handedness[0][0].category_name == "Left":
                    left_gesture = detection_result.gestures[0][0].category_name
                    right_gesture = "None"
                    left_index = 0
                elif detection_result.handedness[0][0].category_name == "Right":
                    right_gesture = detection_result.gestures[0][0].category_name
                    left_gesture = "None"
                    right_index = 0

        if detection_result.hand_landmarks and right_index is not None:
            # self.get_logger().info("Right Hand")
            coords = np.array([[landmark.x * np.shape(annotated_image)[1],
                                landmark.y * np.shape(annotated_image)[0]]
                               for landmark in [detection_result.hand_landmarks[right_index][0],
                                                detection_result.hand_landmarks[right_index][1],
                                                detection_result.hand_landmarks[right_index][2],
                                                detection_result.hand_landmarks[right_index][5],
                                                detection_result.hand_landmarks[right_index][9],
                                                detection_result.hand_landmarks[right_index][14],
                                                detection_result.hand_landmarks[right_index][17]]])
        # now perform the math on the numpy arrays. I think this is faster?
            length = coords.shape[0]
            if length > 0: 
                sum_x = np.sum(coords[:, 0])
                sum_y = np.sum(coords[:, 1])
                self.centroid = np.array([sum_x/length, sum_y/length, 0.0])
    
            # Add bounds checking
            centroid_y = int(self.centroid[1])
            centroid_x = int(self.centroid[0])
            
            if (0 <= centroid_y < self.depth_image.shape[0] and 
                0 <= centroid_x < self.depth_image.shape[1]):
                depth_value = self.depth_image[centroid_y, centroid_x]
                # Filter out invalid depth values (NaN, inf, or zero)
                if np.isfinite(depth_value) and depth_value > 0:
                    self.centroid[2] = depth_value
                else:
                    self.get_logger().warn("Invalid depth value at centroid")
            else:
                self.get_logger().warn(f"Centroid coordinates ({centroid_x}, {centroid_y}) out of depth image bounds ({self.depth_image.shape[1]}, {self.depth_image.shape[0]})")

        self.waypoint.pose.position.x = self.centroid[0]
        self.waypoint.pose.position.y = self.centroid[1]
        self.waypoint.pose.position.z = self.centroid[2]

        text = f"(x: {np.round(self.centroid[0] - self.image_width/2)}, y: {np.round(self.centroid[1] - self.image_height/2)}, z: {np.round(self.centroid[2])})"

        annotated_image = cv.putText(annotated_image, text,
                                     (int(self.centroid[0])-100,
                                      int(self.centroid[1])+40),
                                     cv.FONT_HERSHEY_COMPLEX, 0.5, (255, 255, 255), 1)

        annotated_image = cv.circle(
            annotated_image, (int(self.centroid[0]), int(self.centroid[1])), 10, (255, 255, 255), -1)

        cv_image = self.bridge.cv2_to_imgmsg(
            annotated_image, encoding="rgb8")

        return cv_image, right_gesture

    def process_color_image(self):
        """Process the color image to find the 3D location of the hand's pose."""
        try:

            mp_image = mp.Image(
                image_format=mp.ImageFormat.SRGB, data=self.color_image)

            detection_result = self.mps.landmarker.recognize(mp_image)
            annotated_image = self.mps.draw_landmarks_on_image(
                rgb_image=self.color_image, detection_result=detection_result)

            return annotated_image, detection_result

        except CvBridgeError:
            self.get_logger().error(CvBridgeError)

    def timer_callback(self):
        """Publish the annotated image and the waypoint for the arm"""
        if self.color_image is not None and self.depth_image is not None:
            annotated_image, detection_result = self.process_color_image()
            if annotated_image is not None and detection_result is not None:
                cv_image, right_gesture = self.process_depth_image(
                    annotated_image, detection_result)
                if cv_image is not None:
                    self.cv_image_pub.publish(cv_image)
                    self.right_gesture_pub.publish(String(data=right_gesture))
        
        # publish the waypoint
        self.waypoint.header.stamp = self.get_clock().now().to_msg()
        self.waypoint_pub.publish(self.waypoint)


def main(args=None):
    rclpy.init(args=args)

    handcv = HandCV()

    rclpy.spin(handcv)


if __name__ == '__main__':
    main()

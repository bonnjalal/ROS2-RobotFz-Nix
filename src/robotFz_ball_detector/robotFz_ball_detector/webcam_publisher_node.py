#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import time
import numpy as np


class WebcamPublisher(Node):
    def __init__(self):
        super().__init__("webcam_publisher")
        self.publisher_ = self.create_publisher(
            CompressedImage, "/agent1/image/compressed", 10
        )
        self.bridge_ = CvBridge()
        # self.cap_ = cv2.VideoCapture(0)  # 0 is the default webcam
        self.cap_ = cv2.VideoCapture("http://192.168.1.1/")  # 0 is the default webcam
        if not self.cap_.isOpened():
            self.get_logger().error("Failed to open webcam")
            exit()
        self.timer_ = self.create_timer(0.033, self.publish_image)  # Publish at 30 Hz
        self.get_logger().info("Webcam publisher node started.")

    def publish_image(self):
        ret, frame = self.cap_.read()
        if not ret:
            self.get_logger().warn("Failed to read frame from webcam")
            return

        try:
            # Convert the OpenCV image to a ROS CompressedImage message
            compressed_image_msg = self.bridge_.cv2_to_compressed_imgmsg(frame, "jpeg")

            # Publish the message
            self.publisher_.publish(compressed_image_msg)

            # self.get_logger().info("Image published")
        except Exception as e:
            self.get_logger().error(f"Failed to publish image: {e}")
            return

    def destroy_node(self):
        self.cap_.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    webcam_publisher = WebcamPublisher()
    try:
        rclpy.spin(webcam_publisher)
    except KeyboardInterrupt:
        print("Shutting down webcam publisher")
    finally:
        webcam_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

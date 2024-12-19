#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np
import time


# Assuming 'common.py' is in the same directory, adjust if necessary.
# from .common import *  # if common.py is in a sub-directory, use from my_package import common
# If common.py contains ROS related information, you'll have to adjust that too


Hue_range_max = 179
Saturation_range_max = 255

messages_prefix = "ball_follow"  # replace
detector_publisher_name = "/ball_location_coordinates"


class Point2D:
    def __init__(self, x, y):
        # type: (float, float) -> None
        self.x = x
        self.y = y

    def __repr__(self):
        return "({x}, {y})".format(x=self.x, y=self.y)

    def verbose_str(self):
        # type: () -> str
        return "x = {x}, y = {y}".format(x=self.x, y=self.y)

    @classmethod
    def from_tuple(cls, x_y_tuple):
        # type: (Sequence) -> Point2D
        return cls(x_y_tuple[0], x_y_tuple[1])

    def round_and_convert_values_to_int(self):
        # type: () -> Point2D
        x_int = int(round(self.x))
        y_int = int(round(self.y))

        return Point2D(x_int, y_int)

    def to_tuple(self):
        # type: () -> Tuple
        return tuple((self.x, self.y))


class Circle:
    def __init__(self, center, radius):
        # type: (Point2D, float) -> None
        self.center = center
        self.radius = radius

    def __repr__(self):
        # type: () -> str
        return "Center = {center}, Radius = {radius}".format(
            center=self.center, radius=self.radius
        )

    @classmethod
    def from_x_y_radius(cls, x, y, radius):
        # type: (float, float, float) -> Circle
        return cls(center=Point2D(x, y), radius=radius)

    @classmethod
    def from_x_y_r_sequence(cls, x_y_r):
        # type: (Sequence) -> Circle
        return cls.from_x_y_radius(x=x_y_r[0], y=x_y_r[1], radius=x_y_r[2])

    def to_x_y_r(self):
        # type: () -> Tuple
        return tuple((self.center.x, self.center.y, self.radius))

    def round_and_convert_values_to_int(self):
        # type: () -> Circle
        center_int = self.center.round_and_convert_values_to_int()
        radius_int = int(round(self.radius))

        return Circle(center_int, radius_int)


# calculate mean center and radius of a set of circles
def calculate_mean_of_circles(circles):
    # type: (Sequence) -> Circle
    matrix_height = len(circles)
    matrix_width = 3  # x, y, r
    circles_properties_matrix = np.zeros((matrix_height, matrix_width))

    for circle, i in zip(circles, range(len(circles))):
        circles_properties_matrix[i] = circle.to_x_y_r()

    mean_circle = Circle.from_x_y_r_sequence(circles_properties_matrix.mean(axis=0))

    return mean_circle


# create a mask with the image dimensions, masking everything but the circle
def make_ball_mask(ball_image, ball_circle):
    # type: (np.ndarray, Circle) -> np.ndarray
    mask = np.zeros(shape=ball_image.shape[:2], dtype=np.uint8)
    ball_circle = ball_circle.round_and_convert_values_to_int()
    cv.circle(
        img=mask,
        center=ball_circle.center.to_tuple(),
        radius=ball_circle.radius,
        color=(1),
        thickness=-1,
    )

    return mask


# calculate average color of the region in the image where the circle is
def calculate_ball_mean_color(ball_image, ball_circle):
    # type: (np.ndarray, Circle) -> Tuple
    mask = make_ball_mask(ball_image=ball_image, ball_circle=ball_circle)
    mean_color = cv.mean(src=ball_image, mask=mask)[0:3]

    return mean_color


def put_disc_at_top_left_of_image(img, disc_radius, disc_color, top_left_margin):
    # type: (np.ndarray, float, Sequence, float) -> None
    assert disc_radius > 0 and top_left_margin >= 0

    center_distance = top_left_margin + disc_radius
    circle = Circle(
        center=Point2D(x=center_distance, y=center_distance), radius=disc_radius
    )
    circle_rounded = circle.round_and_convert_values_to_int()
    cv.circle(
        img=img,
        center=circle_rounded.center.to_tuple(),
        radius=circle_rounded.radius,
        color=disc_color,
        thickness=-1,
    )


def display_ball_mean_color_on_image(ball_image, ball_circle):
    # type: (np.ndarray, Circle) -> None
    ball_mean_color = calculate_ball_mean_color(
        ball_image=ball_image, ball_circle=ball_circle
    )
    ball_mean_color_rounded = np.round(ball_mean_color).astype("int")
    b, g, r = ball_mean_color_rounded
    print("The mean color of the ball is: " "R={r}, G={g}, B={b}".format(r=r, g=g, b=b))
    image = ball_image.copy()
    put_disc_at_top_left_of_image(
        img=image,
        disc_radius=50,
        disc_color=ball_mean_color_rounded,
        top_left_margin=50,
    )
    cv.imshow("Ball Image with Ball Detected Color", image)
    cv.waitKey(1)  # waitKey(0) will pause, 1 or 3 shows the image briefly


# convert table of circles properties (x, y, r) to list of circles
def convert_circles_properties_to_list_of_circles(circles_properties):
    # type: (np.ndarray) -> List
    return [Circle.from_x_y_r_sequence(x_y_r) for x_y_r in circles_properties[0]]


# draw the circles on the image
def put_circles_on_image(
    image,
    circles,
    outer_line_color,
    outer_line_thickness,
    center_point_color,
    center_point_radius,
):
    # type: (np.ndarray, Iterable, Sequence, int, Sequence, int) -> None
    for circle in circles:
        # plot circle outer line
        cv.circle(
            img=image,
            center=circle.center.to_tuple(),
            radius=circle.radius,
            color=outer_line_color,
            thickness=outer_line_thickness,
        )
        # plot circle center point
        cv.circle(
            img=image,
            center=circle.center.to_tuple(),
            radius=center_point_radius,
            color=center_point_color,
            thickness=-1,
        )


def put_regular_circles_on_image(image, circles):
    # type: (np.ndarray, Iterable) -> None
    put_circles_on_image(
        image=image,
        circles=circles,
        outer_line_color=(255, 0, 255),
        outer_line_thickness=1,
        center_point_color=(255, 128, 0),
        center_point_radius=1,
    )


def put_bold_circles_on_image(image, circles):
    # type: (np.ndarray, Iterable) -> None
    put_circles_on_image(
        image=image,
        circles=circles,
        outer_line_color=(0, 255, 0),
        outer_line_thickness=4,
        center_point_color=(0, 128, 255),
        center_point_radius=5,
    )


def display_detected_circles_on_image(image, detected_circles):
    # type: (np.ndarray, Iterable) -> None
    detected_circles_ints = [
        circle.round_and_convert_values_to_int() for circle in detected_circles
    ]
    put_regular_circles_on_image(image=image, circles=detected_circles_ints)
    mean_circle_detected = calculate_mean_of_circles(detected_circles)
    mean_circle_detected_ints = mean_circle_detected.round_and_convert_values_to_int()
    put_bold_circles_on_image(image=image, circles=(mean_circle_detected_ints,))

    cv.imshow("", image)
    cv.waitKey(1)


# Class for detecting pink/red ball from published topic images.
# Detection tested only when only one ball is present at any frame.
# Behaviour is unpredicted when there is no ball or there are more than
# one ball.
# Note: colors are in BGR format
class SingleBallDetector(Node):
    _ball_hue_percent = 348.0 / 360
    _ball_hue_deviation_percent = 0.05

    def __init__(
        self,
        ball_color_bgr,
        published_images_topic_name,
        ball_location_in_image_coordinates_publisher_name,
    ):
        # type: (Sequence, str, str) -> None
        super().__init__("ball_detector")
        self.image = None
        self.ball_expected_color = ball_color_bgr
        ball_color_bgr = np.array(ball_color_bgr, dtype="float")
        self.color_weights = ball_color_bgr / ball_color_bgr.sum()

        self.cv_bridge = CvBridge()
        self.image_subscriber = self.create_subscription(
            CompressedImage,
            published_images_topic_name,
            self._on_image_published,
            10,
        )

        self.location_on_image_publisher = self.create_publisher(
            String, ball_location_in_image_coordinates_publisher_name, 10
        )
        self.get_logger().info("Ball detector node started.")

    def _on_image_published(self, data):
        # type: (Image_message) -> None
        cv_image = None

        try:
            cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridgeError: {e}")
            return
        if cv_image is not None:
            output_image = cv_image
            detected_ball, image_with_detected_ball = self.detect_ball(cv_image)
            if detected_ball is not None:
                msg = String()
                msg.data = "x={x},y={y},r={r},h={h},w={w},t={t}".format(
                    x=detected_ball.center.x,
                    y=detected_ball.center.y,
                    r=detected_ball.radius,
                    h=cv_image.shape[0],
                    w=cv_image.shape[1],
                    t=self.get_clock().now().nanoseconds / 1e9,
                )
                self.location_on_image_publisher.publish(msg)
                output_image = image_with_detected_ball
            else:
                self.get_logger().info("No ball found")

            cv.imshow("Image window", output_image)
            cv.waitKey(1)

        else:
            self.get_logger().info("No image")

    # convert img with range 0..255 to 0.0..1.0
    @staticmethod
    def _convert_int_to_float(image):
        # type: (np.ndarray) -> np.ndarray
        return image.astype("float") / 255

    # convert img with range 0.0..1.0 to 0..255
    @staticmethod
    def _convert_float_to_int(image):
        # type: (np.ndarray) -> np.ndarray
        return (image * 255).astype(np.uint8)

    # stretch values to fill the range 0.0..1.0
    @staticmethod
    def _normalize_image(image):
        # type: (np.ndarray) -> np.ndarray
        image = image.astype("float")
        image = image - image.min()
        max_value = image.max()
        if max_value > 0:
            image /= max_value

        return image

    # combine the image's channels to 1 channel using weights,
    # so that the ball has highest contrast
    def _optimize_ball_contrast(self, image):
        # type: (np.ndarray) -> np.ndarray
        weighted_image = image * self.color_weights
        weighted_image = weighted_image.sum(axis=2)
        weighted_image = self._normalize_image(weighted_image)

        return weighted_image

    @staticmethod
    def _make_binary_array_using_bounds(array, lower_bound, upper_bound):
        # type: (np.ndarray, float, float) -> np.ndarray
        binary_array = np.ones(shape=array.shape, dtype=np.uint8)
        if lower_bound < 0 and upper_bound <= 1:
            binary_array[
                np.logical_and(array >= upper_bound, array <= lower_bound + 1)
            ] = 0
        elif lower_bound >= 0:
            binary_array[np.logical_or(array < lower_bound, array > upper_bound)] = 0

        return binary_array

    # make a binary image such that: pixel has a value of 1 in the binary image if a pixel
    # at the same position in the given image has hue in the range calculated at initialization.
    # Otherwise, the pixel will have a value of 0.
    def _filter_image_using_ball_color(self, image):
        # type: (np.ndarray) -> np.ndarray
        hue, saturation, value = cv.split(cv.cvtColor(src=image, code=cv.COLOR_BGR2HSV))
        # type: (np.ndarray, np.ndarray, np.ndarray)

        # scale to 0..1
        hue = hue.astype("float") / Hue_range_max
        hue_max = self._ball_hue_percent + self._ball_hue_deviation_percent
        hue_min = self._ball_hue_percent - self._ball_hue_deviation_percent
        hue_condition_matched = self._make_binary_array_using_bounds(
            array=hue, lower_bound=hue_min, upper_bound=hue_max
        )

        binary_image = np.zeros(shape=image.shape[:2], dtype=np.uint8)
        binary_image[hue_condition_matched == 1] = 1
        binary_image_float = binary_image.astype(np.float64)

        return binary_image_float

    # draw circles on the image. the less prominent circles are called 'weak_circles'.
    # the more prominent circles are called 'strong_circles'.
    @staticmethod
    def _put_circles_on_image(image, weak_circles, strong_circles):
        # type: (np.ndarray, Iterable, Iterable) -> None
        weak_circles_ints = [
            circle.round_and_convert_values_to_int() for circle in weak_circles
        ]
        put_regular_circles_on_image(image=image, circles=weak_circles_ints)
        put_bold_circles_on_image(image=image, circles=strong_circles)

    # detect all the circles in a single-channel image (matrix)
    @staticmethod
    def _detect_circles(image_1_channel):
        # type: (np.ndarray) -> np.ndarray
        return cv.HoughCircles(
            image=SingleBallDetector._convert_float_to_int(image_1_channel),
            method=cv.HOUGH_GRADIENT,
            dp=2.0,
            minDist=10,
            param1=100,
            param2=30,
            minRadius=10,
            maxRadius=40,
        )

    # detect the ball with the specific color in the image
    def detect_ball(self, image):
        # type: (np.ndarray) -> (Circle or None, np.ndarray)
        filtered_color_binary_image = self._filter_image_using_ball_color(image)
        cv.imshow("Filtered Binary Image", filtered_color_binary_image)

        detected_circles = self._detect_circles(filtered_color_binary_image)
        if detected_circles is not None:
            detected_circles = convert_circles_properties_to_list_of_circles(
                detected_circles
            )

        if detected_circles is not None:
            self.get_logger().info(
                "Found {num_of_circles} Circles".format(
                    num_of_circles=str(len(detected_circles))
                )
            )
            mean_circle_detected = calculate_mean_of_circles(detected_circles)
            mean_circle_detected_ints = (
                mean_circle_detected.round_and_convert_values_to_int()
            )
            self._put_circles_on_image(
                image=image,
                weak_circles=detected_circles,
                strong_circles=(mean_circle_detected_ints,),
            )
        else:
            mean_circle_detected_ints = None

        return mean_circle_detected_ints, image


def main(args=None):
    rclpy.init(args=args)
    ball_detector = SingleBallDetector(
        ball_color_bgr=(27, 14, 16),
        published_images_topic_name="/agent1/image/compressed",
        ball_location_in_image_coordinates_publisher_name=messages_prefix
        + detector_publisher_name,
    )
    try:
        rclpy.spin(ball_detector)
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        ball_detector.destroy_node()
        rclpy.shutdown()
    cv.destroyAllWindows()


if __name__ == "__main__":
    main()

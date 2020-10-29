#!/usr/bin/env python2

"""
Use SIFT to detect license plates for susequent reading

Subscribes to /R1/pi_camera/image_raw for image data
Publishes plate readings to /license_plate
When environment variable HIPPO_DEBUG is set and non-zero, publishes annotated images
to /hippo/license_debug
"""

from __future__ import print_function, division

import os

import cv_bridge
import cv2
import rospy
from sensor_msgs import msg

import util

TOPICS = {
    "images": "/R1/pi_camera/image_raw",
    "plates": "/license_plate",
    "debug": "/hippo/license_debug",
}

bridge = cv_bridge.CvBridge()
debug_stream = rospy.Publisher(TOPICS["debug"], msg.Image, queue_size=1)

DEBUG = os.getenv("HIPPO_DEBUG", 0) != 0

plates = {n: util.imread("plates/p{}.png".format(n)) for n in "12345678"}


def detect_plate(ros_image):
    image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    debug_stream.publish(bridge.cv2_to_imgmsg(image, "bgr8"))


if __name__ == "__main__":
    if DEBUG:
        print("Topics:")
        print(TOPICS)

    rospy.init_node("license_detector", anonymous=True)
    subscriber = rospy.Subscriber(TOPICS["images"], msg.Image, detect_plate)
    rospy.spin()

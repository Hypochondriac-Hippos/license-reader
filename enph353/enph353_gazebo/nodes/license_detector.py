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
import rospkg
from sensor_msgs import msg

import util

TOPICS = {
    "images": "/R1/pi_camera/image_raw",
    "plates": "/license_plate",
    "debug": "/hippo/license_debug",
}

DEBUG = os.getenv("HIPPO_DEBUG", 0) != 0
if DEBUG:
    debug_stream = rospy.Publisher(TOPICS["debug"], msg.Image, queue_size=1)

bridge = cv_bridge.CvBridge()
rospack = rospkg.RosPack()
plates = {
    n: util.imread(
        os.path.join(
            rospack.get_path("enph353_gazebo"), "media", "plates", "p{}.png"
        ).format(n)
    )
    for n in "12345678"
}
sift = cv2.xfeatures2d.SIFT_create()
flann = cv2.FlannBasedMatcher({"algorithm": 1, "trees": 5}, {"checks": 50})
plate_kp = {
    n: sift.detectAndCompute(image, mask=None) for n, image in plates.iteritems()
}

MIN_MATCH_COUNT = 10


def detect_plate(ros_image):
    image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    keypoints, descriptors = sift.detectAndCompute(image, mask=None)
    potential_plates = ""
    for plate_n, (_, plate_desc) in plate_kp.iteritems():
        matches = flann.knnMatch(descriptors, plate_desc, k=2)
        matchesMask = [[0, 0] for i in range(len(matches))]

        # Ratio test
        good = []
        for i, (m, n) in enumerate(matches):
            if m.distance / n.distance < 0.7:
                matchesMask[i] = [1, 0]
                good.append(m)
        if len(good) > MIN_MATCH_COUNT:
            potential_plates += plate_n

    if DEBUG:
        if potential_plates == "":
            potential_plates = "None"
        image = cv2.putText(
            image,
            potential_plates,
            (0, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 255, 255),
        )
        debug_stream.publish(bridge.cv2_to_imgmsg(image, "bgr8"))


if __name__ == "__main__":
    if DEBUG:
        print("Topics:")
        print(TOPICS)

    rospy.init_node("license_detector", anonymous=True)
    subscriber = rospy.Subscriber(TOPICS["images"], msg.Image, detect_plate)
    rospy.spin()

#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import message_filters
import cv_bridge

from autoware_msgs.msg import DetectedObjectArray
from autoware_msgs.msg import ImageRect
from sensor_msgs.msg import Image

class TrafficlightClassifier():

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        queue_size = rospy.get_param('~queue_size', 100)

        self.image_pub = rospy.Publisher(
            '~output', Image, queue_size=queue_size)

        sub_red = message_filters.Subscriber(
            '/red_hsv_color_filter/image', Image, queue_size=queue_size)
        sub_blue = message_filters.Subscriber(
            '/blue_hsv_color_filter/image', Image, queue_size=queue_size)
        self.subs = [sub_red, sub_blue]

        if rospy.get_param('~approximate_sync', False):
            slop = rospy.get_param('~slop', 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                fs=self.subs, queue_size=queue_size, slop=slop)
        else:
            sync = message_filters.TimeSynchronizer(
                fs=self.subs, queue_size=queue_size)
            sync.registerCallback(self.callback)

    def callback(self, redmsg, bluemsg):
        red_image = self.bridge.imgmsg_to_cv2(redmsg, desired_encoding='bgr8')
        blue_image = self.bridge.imgmsg_to_cv2(bluemsg, desired_encoding='bgr8')

        red_px_size = len(red_image[red_image > 0.5])
        red_ratio = red_px_size / float(red_image.size)
        print(red_px_size, red_image.size)

        blue_px_size = len(blue_image[blue_image > 0.5])
        blue_ratio = blue_px_size / float(blue_image.size)

        print('blue_ratio, red_ratio: ', blue_ratio, red_ratio)

if __name__ == '__main__':
    rospy.init_node('trafficlight_classification')
    trafficlight_classifier= TrafficlightClassifier()
    rospy.spin()

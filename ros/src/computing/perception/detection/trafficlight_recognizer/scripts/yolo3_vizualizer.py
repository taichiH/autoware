#!/usr/bin/env python

import cv2
import copy
import numpy as np

import rospy
import cv_bridge
from sensor_msgs.msg import Image
from autoware_msgs.msg import DetectedObject, DetectedObjectArray

class Yolo3Vizualizer():

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.width = 1920
        self.height = 1200
        self.image = None

        self.image_pub = rospy.Publisher(
            '/yolo3_vizualizer/output', Image, queue_size=1)

        rospy.Subscriber(
            '/detection/image_detector/croped_objects', DetectedObjectArray, self.callback)
        rospy.Subscriber(
            '/image_raw', Image, self.image_callback)

    def image_callback(self, msg):
        br = self.bridge
        self.image = br.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def callback(self, msg):
        br = self.bridge
        img = copy.copy(self.image)
        for obj in msg.objects:
            img = cv2.rectangle(
                img,(obj.x, obj.y), (obj.x + obj.width, obj.y + obj.height), (255,0,0),3)

        vis_msg = br.cv2_to_imgmsg(img, encoding='bgr8')
        self.image_pub.publish(vis_msg)

if __name__=='__main__':
    rospy.init_node('yolo3_vizualizer')
    yolo3_vizualizer = Yolo3Vizualizer()
    rospy.spin()

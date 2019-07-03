#!/usr/bin/env python

import cv2
import copy
import numpy as np

import rospy
import cv_bridge
from sensor_msgs.msg import Image
from autoware_msgs.msg import DetectedObject, DetectedObjectArray
from kcf_ros.msg import Rect as KcfRect
from jsk_recognition_msgs.msg import RectArray as JskRectArray
from jsk_recognition_msgs.msg import LabelArray as JskLabelArray

class Yolo3Visualizer():

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image = None
        self.check_callback = False
        self.nearest_roi_rect = None
        self.kcf_rect = None
        self.qatm_rect_array = None
        self.text_size = 0.5
        self.text_width = 1

        self.image_pub = rospy.Publisher(
            '~output', Image, queue_size=1)

        rospy.Subscriber(
            '~input_detected_boxes', DetectedObjectArray, self.callback)
        rospy.Subscriber(
            '~input_nearest_roi_rect', KcfRect, self.rect_callback)
        rospy.Subscriber(
            '~input_kcf_rect', KcfRect, self.kcf_callback)
        rospy.Subscriber(
            '~input_qatm_rect_array', JskRectArray, self.qatm_rect_callback)
        rospy.Subscriber(
            '~input_qatm_label_array', JskLabelArray, self.qatm_label_callback)
        rospy.Subscriber(
            '~input_image', Image, self.image_callback)

    def image_callback(self, msg):
        self.check_callback = True
        br = self.bridge
        self.image = br.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def rect_callback(self, msg):
        self.nearest_roi_rect = msg

    def kcf_callback(self, msg):
        self.kcf_rect = msg

    def qatm_rect_callback(self, msg):
        self.qatm_rect_array = msg

    def qatm_label_callback(self, msg):
        self.qatm_label_array = msg

    def callback(self, msg):
        if not self.check_callback:
            rospy.logwarn('no image comming')
            return

        br = self.bridge
        img = copy.copy(self.image)

        if not self.nearest_roi_rect:
            rospy.logwarn('nearest_roi_rect is None')
            return

        img = cv2.rectangle(
            img, (self.nearest_roi_rect.x, self.nearest_roi_rect.y),
            (self.nearest_roi_rect.x + self.nearest_roi_rect.width,
             self.nearest_roi_rect.y + self.nearest_roi_rect.height),
            (0,0,255),2)
        cv2.putText(
            img, 'nearest_trafficlight_roi',
            (self.nearest_roi_rect.x, self.nearest_roi_rect.y),
            cv2.FONT_HERSHEY_SIMPLEX, self.text_size, (0,0,255), self.text_width, cv2.LINE_AA)

        for obj in msg.objects:
            x = obj.x + self.nearest_roi_rect.x
            y = obj.y + self.nearest_roi_rect.y

            cv2.putText(
                img, 'yolo detected box',
                (x, y + obj.height),
                cv2.FONT_HERSHEY_SIMPLEX, self.text_size, (255,0,0), self.text_width, cv2.LINE_AA)
            img = cv2.rectangle(
                img, (x, y), (x + obj.width, y + obj.height), (255,0,0),2)

        if not self.qatm_rect_array:
            rospy.logwarn('qatm_rect_array is None')
            return
        if not self.kcf_rect:
            rospy.logwarn('kcf_rect is None')
            return

        for box, label in zip(self.qatm_rect_array.rects, self.qatm_label_array.labels):
            qatm_x = self.kcf_rect.x + box.x
            qatm_y = self.kcf_rect.y + box.y
            cv2.putText(
                img, label.name,
                (qatm_x, qatm_y),
                cv2.FONT_HERSHEY_SIMPLEX, self.text_size, (0,255,0), self.text_width, cv2.LINE_AA)

            img = cv2.rectangle(
                img, (qatm_x, qatm_y), (qatm_x + box.width, qatm_y + box.height), (0,255,0),2)

        vis_msg = br.cv2_to_imgmsg(img, encoding='bgr8')
        self.image_pub.publish(vis_msg)

if __name__=='__main__':
    rospy.init_node('yolo3_visualizer')
    yolo3_visualizer = Yolo3Visualizer()
    rospy.spin()

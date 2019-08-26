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

        self.font_size = 0.3
        self.ratio_thresh = rospy.get_param('~ratio_thresh', 0.02)

        self.image_pub = rospy.Publisher(
            '~output', Image, queue_size=queue_size)
        self.debug_pub = rospy.Publisher(
            '~debug', Image, queue_size=queue_size)

        sub_original = message_filters.Subscriber(
            '~input_original', Image, queue_size=queue_size)
        sub_red = message_filters.Subscriber(
            '~input_red', Image, queue_size=queue_size)
        sub_blue = message_filters.Subscriber(
            '~input_blue', Image, queue_size=queue_size)
        sub_black = message_filters.Subscriber(
            '~input_black', Image, queue_size=queue_size)
        self.subs = [sub_red, sub_blue, sub_black, sub_original]

        if rospy.get_param('~approximate_sync', False):
            slop = rospy.get_param('~slop', 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                fs=self.subs, queue_size=queue_size, slop=slop)
        else:
            sync = message_filters.TimeSynchronizer(
                fs=self.subs, queue_size=queue_size)
            sync.registerCallback(self.callback)

    def add_info(self, image, text, ratio):
        text_img = np.zeros(image.shape,dtype=np.uint8)
        cv2.putText(
            text_img, text, (10,10), cv2.FONT_HERSHEY_SIMPLEX, self.font_size, (255,255,255), 1, cv2.LINE_AA)
        cv2.putText(
            text_img, str(round(ratio, 5)), (10,20), cv2.FONT_HERSHEY_SIMPLEX, self.font_size, (255,255,255), 1, cv2.LINE_AA)
        dst = cv2.hconcat([text_img, image])
        return dst

    def filtering(self, image):
        kernel = np.ones((5,5),np.uint8)
        dst =cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
        px_size = len(dst[dst > 0.5])
        ratio = px_size / float(dst.size)
        return dst, ratio

    def callback(self, redmsg, bluemsg, blackmsg, originalmsg):
        original_image = self.bridge.imgmsg_to_cv2(originalmsg, desired_encoding='bgr8')
        red_image = self.bridge.imgmsg_to_cv2(redmsg, desired_encoding='bgr8')
        blue_image = self.bridge.imgmsg_to_cv2(bluemsg, desired_encoding='bgr8')
        black_image = self.bridge.imgmsg_to_cv2(blackmsg, desired_encoding='bgr8')
        images = [red_image, blue_image, black_image]
        texts = ['red', 'blue', 'black']

        ratios = []
        for i, image in enumerate(images):
            dst, ratio = self.filtering(image)
            images[i] = dst
            ratios.append(ratio)

        for i, image in enumerate(images):
            images[i] = self.add_info(image, texts[i], ratios[i])

        text_img = np.zeros(original_image.shape,dtype=np.uint8)
        cv2.putText(
            text_img, 'original', (10,10), cv2.FONT_HERSHEY_SIMPLEX, self.font_size, (255,255,255), 1, cv2.LINE_AA)
        original_image = cv2.hconcat([text_img, original_image])


        concated_image = cv2.vconcat([original_image, images[0]])
        concated_image = cv2.vconcat([concated_image, images[1]])
        concated_image = cv2.vconcat([concated_image, images[2]])

        if ratios[1] > self.ratio_thresh:
            result = 'go'
            color = (255,0,0)
        elif ratios[0] > self.ratio_thresh:
            result = 'stop'
            color = (0,0,255)
        else:
            result = 'unknown'
            color = (255,255,255)

        output_image = np.full(images[0].shape, color, dtype=np.uint8)
        cv2.putText(
            output_image, result, (30,10), cv2.FONT_HERSHEY_SIMPLEX, self.font_size, (0,0,0), 1, cv2.LINE_AA)

        concated_image = cv2.vconcat([concated_image, output_image])
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(output_image, encoding='bgr8'))
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(concated_image, encoding='bgr8'))

if __name__ == '__main__':
    rospy.init_node('trafficlight_classification')
    trafficlight_classifier= TrafficlightClassifier()
    rospy.spin()

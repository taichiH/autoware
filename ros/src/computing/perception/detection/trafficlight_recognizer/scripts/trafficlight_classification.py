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

        self.font_size = 0.8
        self.buffer_size = 5
        self.resize_size = (150,150)
        self.ratio_thresh = rospy.get_param('~ratio_thresh', 0.02)
        self.ratios_buffer = []
        self.background = (20, 20, 20)

        template_path = rospy.get_param('~template_path')
        self.template_image = cv2.cvtColor(cv2.imread(template_path), cv2.COLOR_BGR2GRAY)

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
        sub_yellow = message_filters.Subscriber(
            '~input_yellow', Image, queue_size=queue_size)
        sub_black = message_filters.Subscriber(
            '~input_black', Image, queue_size=queue_size)
        self.subs = [sub_red, sub_blue, sub_yellow, sub_black, sub_original]

        if rospy.get_param('~approximate_sync', False):
            slop = rospy.get_param('~slop', 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                fs=self.subs, queue_size=queue_size, slop=slop)
        else:
            sync = message_filters.TimeSynchronizer(
                fs=self.subs, queue_size=queue_size)
            sync.registerCallback(self.callback)


    def image_filter(self, images, original_image):
        ratios = [[] for i in range(len(images))]
        filtered_images = [[] for i in range(len(images))]

        for i, image in enumerate(images):
            # erosion -> dilation
            kernel = np.ones((3,3),np.uint8)
            dst = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
            ratios[i] = len(dst[dst > 0.5]) / float(dst.size)

        return ratios, filtered_images

    def moving_filter(self, ratios):
        if len(self.ratios_buffer) > self.buffer_size:
            self.ratios_buffer.pop(0)
        self.ratios_buffer.append(ratios)
        ratios_buffer_arr = np.array(self.ratios_buffer)
        ratios = ratios_buffer_arr.mean(axis=0)
        return ratios

    def concat_text_and_image(self, image, filtered_image, text, ratio):
        text_img = np.full(image.shape, self.background, dtype=np.uint8)
        cv2.putText(
            text_img, text, (10,30), cv2.FONT_HERSHEY_SIMPLEX, self.font_size, (255,255,255), 1, cv2.LINE_AA)
        cv2.putText(
            text_img, str(round(ratio, 5)), (10,60), cv2.FONT_HERSHEY_SIMPLEX, self.font_size, (255,255,255), 1, cv2.LINE_AA)
        dst = cv2.hconcat([text_img, image])
        dst = cv2.hconcat([dst, filtered_image])
        return dst

    def create_vis_image(self, original_image, images, filtered_images, ratios, result, color):
        texts = ['red', 'blue', 'yellow', 'nonblack']
        for i, image in enumerate(images):
            image = cv2.resize(image, self.resize_size)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            image = cv2.bitwise_and(original_image, original_image, mask=image)

            filtered_image = cv2.resize(filtered_images[i], self.resize_size)
            filtered_image = cv2.cvtColor(filtered_image, cv2.COLOR_GRAY2BGR)
            images[i] = self.concat_text_and_image(image,
                                                   filtered_image,
                                                   texts[i],
                                                   ratios[i])

        text_img = np.full(original_image.shape, self.background, dtype=np.uint8)
        cv2.putText(
            text_img, 'original', (10,30), cv2.FONT_HERSHEY_SIMPLEX, self.font_size, (255,255,255), 1, cv2.LINE_AA)
        original_image = cv2.hconcat([text_img, original_image])
        original_image = cv2.hconcat([text_img, original_image])

        concatenated_image = cv2.vconcat([original_image, images[0]])
        for i in range(1, len(images)):
            concatenated_image = cv2.vconcat([concatenated_image, images[i]])

        output_image = np.full(images[0].shape, color, dtype=np.uint8)
        cv2.putText(
            output_image, result, (10,50), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0,0,0), 3, cv2.LINE_AA)
        concatenated_image = cv2.vconcat([concatenated_image, output_image])
        return concatenated_image

    def make_decision(self, ratios):
        if ratios[1] > self.ratio_thresh:
            result = 'go'
            color = (255,0,0)
        elif ratios[0] > self.ratio_thresh:
            result = 'stop'
            color = (0,0,255)
        else:
            result = 'unknown'
            color = (255,255,255)
        return result, color

    def callback(self, redmsg, bluemsg, yellowmsg, blackmsg, originalmsg):
        original_image = self.bridge.imgmsg_to_cv2(originalmsg, desired_encoding='bgr8')
        red_image = self.bridge.imgmsg_to_cv2(redmsg, desired_encoding='bgr8')
        blue_image = self.bridge.imgmsg_to_cv2(bluemsg, desired_encoding='bgr8')
        yellow_image = self.bridge.imgmsg_to_cv2(yellowmsg, desired_encoding='bgr8')
        black_image = self.bridge.imgmsg_to_cv2(blackmsg, desired_encoding='bgr8')

        images = [red_image, blue_image, yellow_image, black_image]
        ratios, filtered_images = self.image_filter(images, original_image)

        ratios = self.moving_filter(ratios)
        result, color = self.make_decision(ratios)
        original_image = cv2.resize(original_image, self.resize_size)

        concatenated_image = self.create_vis_image(original_image, filtered_images, ratios, result, color)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(concatenated_image, encoding='bgr8'))

if __name__ == '__main__':
    rospy.init_node('trafficlight_classification')
    trafficlight_classifier= TrafficlightClassifier()
    rospy.spin()

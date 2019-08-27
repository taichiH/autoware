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
        self.detector = cv2.xfeatures2d.SIFT_create()

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

    def sift_detector(self, image):
        if self.template_image is None:
            return None

        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        print(image.shape)
        print(self.template_image.shape)

        # self.detector = cv2.xfeatures2d.SIFT_create()
        detector = cv2.xfeatures2d.SIFT_create()
        kp1, des1 = detector.detectAndCompute(self.template_image, None)
        kp2, des2 = detector.detectAndCompute(image, None)
        bf = cv2.BFMatcher()
        matches = bf.knnMatch(des1, des2, k=2)
        good = []
        match_param = 0.6
        for m,n in matches:
            if m.distance < match_param*n.distance:
                good.append([m])
        result_image = cv2.drawMatchesKnn(self.template_image,kp1,input_image,kp2,good, None,flags=2)
        return result_image

    def hough_line(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edge_image = cv2.Canny(gray, 100, 200)

        # ret, thresh = cv2.threshold(edge_image, 127,255, 0)
        # dst, contours, hierarchy = cv2.findContours(
        #     thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # if len(contours) > 1:
        #     rospy.loginfo('%d instances exist' %(len(contours)))

        # debug_img = np.zeros((edge_image.shape[0], edge_image.shape[1], 3), dtype=np.uint8)
        # for ins, contour in enumerate(contours):
        #     for i in range(contours[ins].shape[0]):
        #         debug_img[contours[0][i,0,1], contours[0][i,0,0], :] = [255,0,0]

        return edge_image

    def filtering(self, images, original_image):
        ratios = [[] for i in range(len(images))]
        dst_images = [[] for i in range(len(images))]
        hough_images = [[] for i in range(len(images))]

        for i, image in enumerate(images):
            # erosion -> dilation
            kernel = np.ones((3,3),np.uint8)
            dst = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
            ratio = len(dst[dst > 0.5]) / float(dst.size)
            dst_images[i] = dst
            ratios[i] = ratio

            hough_images[i] = self.hough_line(dst)

        return dst_images, ratios, hough_images

    def moving_filter(self, ratios):
        if len(self.ratios_buffer) > self.buffer_size:
            self.ratios_buffer.pop(0)
        self.ratios_buffer.append(ratios)
        ratios_buffer_arr = np.array(self.ratios_buffer)
        ratios = ratios_buffer_arr.mean(axis=0)
        return ratios

    def concat_text_image(self, image, hough_image, text, ratio):
        text_img = np.full(image.shape, self.background, dtype=np.uint8)
        cv2.putText(
            text_img, text, (10,30), cv2.FONT_HERSHEY_SIMPLEX, self.font_size, (255,255,255), 1, cv2.LINE_AA)
        cv2.putText(
            text_img, str(round(ratio, 5)), (10,60), cv2.FONT_HERSHEY_SIMPLEX, self.font_size, (255,255,255), 1, cv2.LINE_AA)
        dst = cv2.hconcat([text_img, image])
        dst = cv2.hconcat([dst, hough_image])
        return dst

    def create_vis_image(self, original_image, images, hough_images, ratios, result, color):
        texts = ['red', 'blue', 'yellow', 'nonblack']
        for i, image in enumerate(images):
            image = cv2.resize(image, self.resize_size)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            image = cv2.bitwise_and(original_image, original_image, mask=image)

            hough_image = cv2.resize(hough_images[i], self.resize_size)
            hough_image = cv2.cvtColor(hough_image, cv2.COLOR_GRAY2BGR)
            images[i] = self.concat_text_image(image,
                                               hough_image,
                                               texts[i],
                                               ratios[i])

        text_img = np.full(original_image.shape, self.background, dtype=np.uint8)
        cv2.putText(
            text_img, 'original', (10,30), cv2.FONT_HERSHEY_SIMPLEX, self.font_size, (255,255,255), 1, cv2.LINE_AA)
        original_image = cv2.hconcat([text_img, original_image])
        original_image = cv2.hconcat([text_img, original_image])

        concated_image = cv2.vconcat([original_image, images[0]])
        for i in range(1, len(images)):
            concated_image = cv2.vconcat([concated_image, images[i]])

        output_image = np.full(images[0].shape, color, dtype=np.uint8)
        cv2.putText(
            output_image, result, (10,50), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0,0,0), 3, cv2.LINE_AA)
        concated_image = cv2.vconcat([concated_image, output_image])
        return concated_image

    def decision_stop_or_go(self, ratios):
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
        images, ratios, hough_images = self.filtering(images, original_image)

        ratios = self.moving_filter(ratios)
        result, color = self.decision_stop_or_go(ratios)
        original_image = cv2.resize(original_image, self.resize_size)

        concated_image = self.create_vis_image(original_image, images, hough_images, ratios, result, color)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(concated_image, encoding='bgr8'))

if __name__ == '__main__':
    rospy.init_node('trafficlight_classification')
    trafficlight_classifier= TrafficlightClassifier()
    rospy.spin()

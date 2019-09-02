#!/usr/bin/env python

import cv2
import copy
import numpy as np

import rospy
import cv_bridge
from sensor_msgs.msg import Image

class ArrowRecognition():

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        template_path = rospy.get_param('~template_path', './template.jpg')
        self.binary_thresh = rospy.get_param('~binary_thresh', 127)
        self.pair_thresh = rospy.get_param('~pair_thresh', 3)
        self.use_canny = rospy.get_param('~use_canny', False)

        self.template = self.load_template(template_path)
        rospy.Subscriber('~input', Image, self.callback)

    def load_template(self, path):
        template = cv2.imread(path)
        template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
        ret, template = cv2.threshold(template, self.binary_thresh, 255, 0)
        return template


    def draw_contours(self, image, contours):
        debug_img = copy.copy(image)
        debug_img = cv2.cvtColor(debug_img, cv2.COLOR_GRAY2BGR)

        for ins, contour in enumerate(contours):
            for points in contours:
                for point in points:
                    point = point[0]
                    if point[0] > debug_img.shape[1] or point[1] > debug_img.shape[0] or \
                       point[0] < 0 or point[1] < 0:
                        continue

                    debug_img[point[1], point[0], :] = [255, 0, 0]

        return debug_img

    def get_shape(self, thresh_img):
        # dst, contours, hierarchy = cv2.findContours(
        #     thresh_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        dst, contours, hierarchy = cv2.findContours(
            thresh_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 1:
            rospy.loginfo('%d instances exist' %(len(contours)))

        return contours, hierarchy

    def callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.namedWindow('image_input', cv2.WINDOW_NORMAL)
        cv2.imshow('image_input', image)

        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        if self.use_canny:
            print('canny')
            image = cv2.Canny(image, 100, 200)
            template_image = cv2.Canny(self.template, 100, 200)
        else:
            print('binary thresh')
            ret, image = cv2.threshold(image, self.binary_thresh, 255, 0)
            template_ret, template_image = cv2.threshold(self.template, self.binary_thresh, 255, 0)

            kernel = np.ones((5,5),np.uint8)

            # image = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
            image = cv2.dilate(image,kernel,iterations=1)
            template_image = cv2.dilate(template_image,kernel,iterations=1)


        template_contours, _ = self.get_shape(template_image)
        image_contours, _ = self.get_shape(image)

        image_debug = copy.copy(image)
        image_debug = cv2.cvtColor(image_debug, cv2.COLOR_GRAY2BGR)
        template_debug = copy.copy(template_image)
        template_debug = cv2.cvtColor(template_debug, cv2.COLOR_GRAY2BGR)

        print("len(image_contours): ", len(image_contours))
        print("len(template_contours): ", len(template_contours))

        index_pairs = []
        for i, image_cnt in enumerate(image_contours):
            for j, template_cnt in enumerate(template_contours):
                ret = cv2.matchShapes(image_cnt, template_cnt, 1, 0.0)
                if ret < self.pair_thresh:
                    print(i, j, ret)
                    index_pairs.append((i, j, ret))

        for incr, index_pair in enumerate(index_pairs):
            color = (np.random.randint(100,255), np.random.randint(100,255), np.random.randint(100,255))
            i, j, likelihood = index_pair

            image_debug = cv2.drawContours(image_debug, image_contours, i, color, 1)

            text = str(i) + ', ' + str(j) + ', ' + str(round(likelihood, 3))
            # cv2.putText(image_debug, text, (10, 30*(incr+1)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 1, cv2.LINE_AA)

        for i in range(len(template_contours)):
            color = (np.random.randint(100,255), np.random.randint(100,255), np.random.randint(100,255))
            template_debug = cv2.drawContours(template_debug, template_contours, i, color, 1)

        cv2.namedWindow('image_debug', cv2.WINDOW_NORMAL)
        cv2.imshow('image_debug', image_debug)

        cv2.namedWindow('template_debug', cv2.WINDOW_NORMAL)
        cv2.imshow('template_debug', template_debug)

        cv2.waitKey()



if __name__=='__main__':
    rospy.init_node('arrow_recognition_sample')
    arrow_recognition = ArrowRecognition()
    rospy.spin()

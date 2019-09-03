#!/usr/bin/env python

import sys
import traceback

import cv2
import copy
import numpy as np

import rospy
import cv_bridge
from sensor_msgs.msg import Image
from jsk_recognition_msgs.msg import Rect, RectArray
from jsk_recognition_msgs.msg import ClassificationResult

class ArrowRecognition():

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        template_path = rospy.get_param('~template_path', './template.jpg')
        self.binary_thresh = rospy.get_param('~binary_thresh', 127)
        self.pair_thresh = rospy.get_param('~pair_thresh', 3)
        self.area_thresh = rospy.get_param('~area_thresh', 400)
        self.dilate_kernel_size = rospy.get_param('~dilate_kernel_size', 7)
        self.colors = [(np.random.randint(100,255),
                        np.random.randint(100,255),
                        np.random.randint(100,255)) for i in range(1000)]
        self.template = self.load_template(template_path)

        self.debug_pub = rospy.Publisher('~debug', Image, queue_size=1)
        self.rects_pub = rospy.Publisher('~output/rect', RectArray, queue_size=1)
        self.class_pub = rospy.Publisher('~output/class', ClassificationResult, queue_size=1)

        rospy.Subscriber('~input', Image, self.callback)

    def load_template(self, path):
        try:
            template = cv2.imread(path)
            if template is None:
                error_txt = 'Could not find a image: ' + path
                raise ValueError(error_txt)
            template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
            ret, template = cv2.threshold(template, self.binary_thresh, 255, 0)
            return template
        except:
            traceback.print_exc()
            sys.exit()


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

    def get_shape(self, img):
        kernel = np.ones((self.dilate_kernel_size, self.dilate_kernel_size),np.uint8)
        img = cv2.dilate(img, kernel, iterations=1)

        dst, contours, hierarchy = cv2.findContours(
            img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        return img, contours, hierarchy

    def calc_arrow_direction(self, image, x, y, width, height):
        top     = slice(y                , int(y+height*0.5))
        bottom  = slice(int(y+height*0.5), y+height         )
        left    = slice(x                , int(x+width*0.5) )
        right   = slice(int(x+width*0.5) , x+width          )
        lt_cell = image[top,left]
        lb_cell = image[bottom,left]
        rt_cell = image[top,right]
        rb_cell = image[bottom,right]
        lt_area = ('lt', len(lt_cell[lt_cell > 0.5]))
        lb_area = ('lb', len(lb_cell[lb_cell > 0.5]))
        rt_area = ('rt', len(rt_cell[rt_cell > 0.5]))
        rb_area = ('rb', len(rb_cell[rb_cell > 0.5]))

        area_lst = sorted(
            [lt_area, lb_area, rt_area, rb_area],
            key = lambda x : x[1], reverse=True)

        area_array = np.array(area_lst)
        cells = area_array[:2, :1]
        cells = cells.reshape(2)

        if 'lt' in cells and 'rt' in cells:
            return 'top'
        elif 'lb' in cells and 'rb' in cells:
            return 'bottom'
        elif 'lt' in cells and 'lb' in cells:
            return 'left'
        elif 'rt' in cells and 'rb' in cells:
            return 'right'
        else:
            return 'unknown'

    def callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        image, image_contours, _ = self.get_shape(image)
        template_image, template_contours, _ = self.get_shape(self.template)


        image_debug = cv2.cvtColor(copy.copy(image), cv2.COLOR_GRAY2BGR)

        index_pairs = []
        for i, image_cnt in enumerate(image_contours):
            for j, template_cnt in enumerate(template_contours):
                ret = cv2.matchShapes(image_cnt, template_cnt, 1, 0.0)
                area = cv2.contourArea(image_cnt)
                if ret < self.pair_thresh and area > self.area_thresh:
                    index_pairs.append((i, j, ret, area))

        rect_array_msg = RectArray()
        class_msg = ClassificationResult()

        bboxes = []
        index_pairs = sorted(index_pairs, key=lambda x: x[2])
        for incr, index_pair in enumerate(index_pairs):
            color = self.colors[incr]
            i, j, likelihood, area = index_pair
            image_debug = cv2.drawContours(image_debug, image_contours, i, color, 2)
            bboxes.append(cv2.boundingRect(image_contours[i]))

        for x, y, width, height in list(set(bboxes)):
            rect_msg = Rect()
            image_debug = cv2.rectangle(
                image_debug, (x, y), (x + width, y + height), color, 1)
            direction = self.calc_arrow_direction(image, x, y, width, height)
            cv2.putText(
                image_debug,
                str(round(likelihood, 3)),
                (x, y-10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5, color, 1, cv2.LINE_AA)
            cv2.putText(
                image_debug,
                direction, (x, y-30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5, color, 1, cv2.LINE_AA)
            class_msg.label_names.append(direction)
            class_msg.label_proba.append(1)

            rect_msg.x, rect_msg.y, rect_msg.width, rect_msg.height = x, y, width, height
            rect_array_msg.rects.append(rect_msg)


        template_debug = copy.copy(template_image)
        template_debug = cv2.cvtColor(template_debug, cv2.COLOR_GRAY2BGR)
        for i in range(len(template_contours)):
            template_debug = cv2.drawContours(template_debug, template_contours, i, color[i], 2)

        height_diff = image_debug.shape[0] - template_debug.shape[0]
        pad_image = np.zeros((height_diff, template_debug.shape[1], 3), dtype=np.uint8)
        template_debug = cv2.vconcat([pad_image, template_debug])


        cv2.line(template_debug, (0,0), (0, template_debug.shape[0]), (0,0,200), 1,8)
        image_debug = cv2.hconcat([image_debug, template_debug])

        imgmsg = self.bridge.cv2_to_imgmsg(image_debug, encoding='bgr8')
        imgmsg.header = msg.header
        rect_array_msg.header = msg.header
        class_msg.header = msg.header

        self.debug_pub.publish(imgmsg)
        self.rects_pub.publish(rect_array_msg)
        self.class_pub.publish(class_msg)

        cv2.waitKey(50)

if __name__=='__main__':
    rospy.init_node('arrow_recognition_sample')
    arrow_recognition = ArrowRecognition()
    rospy.spin()

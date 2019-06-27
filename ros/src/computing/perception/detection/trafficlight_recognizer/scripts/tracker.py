#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import message_filters
import cv_bridge

from autoware_msgs.msg import DetectedObjectArray
from autoware_msgs.msg import ImageRect
from sensor_msgs.msg import Image

class Tracking():

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()

        self.algorithm = rospy.get_param('~algorithm')
        self.create_tracker()

        self.prev_id = 0
        self.rect_flag = False
        self.missing_box = False
        self.nearest_info = ImageRect()
        self.prev_box = None

        queue_size = rospy.get_param('~queue_size', 1)
        self.image_pub = rospy.Publisher(
            '~output', Image, queue_size=queue_size)

        rospy.Subscriber(
            '/feat_proj_bbox/nearest_info', ImageRect, self.image_rect_callback)

        sub_nearest_image = message_filters.Subscriber(
            '/image_raw', Image, queue_size=queue_size)
        sub_box = message_filters.Subscriber(
            '/detection/image_detector/objects', DetectedObjectArray, queue_size=queue_size)
        self.subs = [sub_nearest_image, sub_box]

        if rospy.get_param('~approximate_sync', False):
            slop = rospy.get_param('~slop', 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                fs=self.subs, queue_size=queue_size, slop=slop)
        else:
            sync = message_filters.TimeSynchronizer(
                fs=self.subs, queue_size=queue_size)
            sync.registerCallback(self.callback)


    def get_nearest_box(self, boxmsg, box_on_roi, w, h):
        min_distance = 24 ** 24
        center = np.array([w*0.5, h*0.5])

        for obj in boxmsg.objects:
            box = (obj.x, obj.y, obj.width, obj.height)
            distance = np.linalg.norm(np.array([obj.x, obj.y]) - center)
            if distance < min_distance:
                box_on_roi = box
                min_distance = distance
                self.prev_box = box_on_roi

        x = self.nearest_info.x + box_on_roi[0]
        y = self.nearest_info.y + box_on_roi[1]
        w = box_on_roi[2]
        h = box_on_roi[3]
        init_box = (x, y, w, h)
        return init_box


    def create_tracker(self):
        if self.algorithm == 'kcf':
            self.tracker = cv2.TrackerKCF_create()
        elif self.algorithm == 'tld':
            self.tracker = cv2.TrackerTLD_create()
        elif self.algorithm == 'goturn':
            self.tracker = cv2.TrackerGOTURN_create()
        else:
            rospy.logerr(rospy.get_param('algorithm') + 'is not implemented')
            return


    # no header on message
    def image_rect_callback(self, msg):
        self.rect_flag = True
        self.nearest_info = msg


    def put_text(self, image, text, pt):
        cv2.putText(
            image, text, pt, cv2.FONT_HERSHEY_SIMPLEX, 3, (0,255,0), 3, cv2.LINE_AA)


    def callback(self, imgmsg, boxmsg):
        if not self.rect_flag:
            return

        if len(boxmsg.objects) == 0:
            return

        image = self.bridge.imgmsg_to_cv2(imgmsg, desired_encoding='bgr8')
        signal_id = self.nearest_info.score

        if self.prev_id != signal_id or self.missing_box:
            init_box = self.get_nearest_box(
                boxmsg, self.prev_box, image.shape[1], image.shape[0])
            cv2.rectangle(image,
                          (int(init_box[0]), int(init_box[1])),
                          (int(init_box[0] + init_box[2]), int(init_box[1] + init_box[3])),
                          (255,0,0), 2, 1)
            self.create_tracker() # re-create tracker every time
            self.tracker.init(image, init_box)
            self.prev_id = signal_id

        # update tracker
        track, bbox = self.tracker.update(image)
        if track:
            cv2.rectangle(image,
                          (int(bbox[0]), int(bbox[1])),
                          (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3])),
                          (0,255,0), 2, 1)
            self.put_text(image, "Tracking", (10, 300))
            self.missing_box = False
        else :
            self.put_text(image, "Missing", (10, 300))
            self.missing_box = True

        self.put_text(image, self.algorithm, (10, 150))

        vis_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        self.image_pub.publish(vis_msg)


if __name__ == '__main__':
    rospy.init_node('tracking')
    tracking = Tracking()
    rospy.spin()

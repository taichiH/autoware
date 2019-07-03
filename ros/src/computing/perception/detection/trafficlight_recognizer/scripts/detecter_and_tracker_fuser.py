#!/usr/bin/env python

import sys
from multiprocessing import Process, Lock

import rospy
from autoware_msgs.msg import DetectedObject, DetectedObjectArray
from kcf_ros.msg import Rect as KcfRect

class Fuser():

    def __init__(self):
        self.mutex = Lock()

        self.cnt = 0
        self.signal_changed = False
        self.detected = False

        self.nearest_roi_rect_buf = []
        self.nearest_roi_rect_queue_size = rospy.get_param('~rect_queue_size', 1000)

        self.tracker_weight = 0
        self.detecter_weight = 0

        rospy.Subscriber('~input_nearest_roi_rect', 1, KcfRect, self.nearest_roi_rect_callback)
        rospy.Subscriber('~input_detection_result', 1, DetectedObjectArray, self.detecter_callback)
        rospy.Subscriber('~input_tracking_result', 1, KcfRect, self.tracker_callback)

    def nearest_roi_rect_callback(self, msg):
        if cnt > self.nearest_roi_rect_queue_size:
            self.nearest_roi_rect_buf.pop()
        self.nearest_roi_rect_queue.append(msg)
        self.cnt += 1

    def get_nearest_stamp_index(self, msg):
        prev_time_diff = 0
        for i, rect in enumerate(self.nearest_roi_rect_queue[::-1]):
            time_diff = rect.header.stamp - msg.header.stamp
            if time_diff < 0:
                nearest_stamp_index = i if time_diff < prev_time_diff else i-1
                break
            prev_time_diff = time_diff
        return nearest_stamp_index

    def calc_detecter_prob(self, detecter_msg, rect_msg):
        # TODO
        pass

    def calc_tracker_prob(self, detecter_msg, rect_msg):
        # TODO
        pass

    def fusion(self, detecter_result, tracker_result):
        # TODO
        pass

    def detecter_callback(self, msg):
        with self.mutex:
            self.detected = True
            nearest_stamp_index = self.get_nearest_stamp_index(msg)
            rect_msg = self.nearest_roi_rect_queue[nearest_stamp_index]

            # calc detecter posterior probability
            self.detecter_weight = self.calc_detecter_prob(msg, rect_msg)
            self.detecter_results.append(detecter_weight)

    def tracker_callback(self, msg):
        with self.mutex:
            nearest_stamp_index = self.get_nearest_stamp_index(msg)
            rect_msg = self.nearest_roi_rect_queue[nearest_stamp_index]

            # when signal changed, clear tracker buffer
            if msg.changed:
                self.signal_changed = msg.changed
                self.tracker_weight = 0

            # calc tracker posterior probability
            self.tracker_weight = self.calc_tracker_prob(msg, rect_msg)
            self.tracker_results.append(tracker)


if __name__=='__main__':
    rospy.init_node('detecter_and_tracker_fuser')
    fuser = Fuser()
    rospy.spin()

#!/usr/bin/env python
# -*- coding: utf-8 -*-

from cv_bridge import CvBridge
import numpy as np

import rospy
import message_filters

from sensor_msgs.msg import Image
from autoware_msgs.msg import StampedRoi
from sensor_msgs.msg import RegionOfInterest

import chainer
from chainercv.links import SSD300

class TLRSSDDetector():

    def __init__(self):
        self.approximate_sync = rospy.get_param('~approximate_sync', True)
        self.gpu = rospy.get_param("~gpu", -1)
        self.cv_bridge = CvBridge()

        # load model
        self.label_names = ['trafficlight']
        rospy.loginfo("Loaded %d labels" % len(self.label_names))
        model_path = rospy.get_param("~model_path", None)
        model_class = SSD300
        self.model = model_class(n_fg_class=len(self.label_names),
                                 pretrained_model=model_path)
        if self.gpu >= 0:
            chainer.cuda.get_device_from_id(self.gpu).use()
            self.model.to_gpu()
        rospy.loginfo("Loaded model: %s" % model_path)


        self.stamped_roi_pub = rospy.Publisher(
            '~output_stamped_roi', StampedRoi, queue_size=1)

        queue_size = rospy.get_param('~queue_size', 100)
        sub_image = message_filters.Subscriber(
            '~input_image', Image, queue_size=queue_size)
        sub_stamped_roi = message_filters.Subscriber(
            '~input_stamped_roi', StampedRoi, queue_size=queue_size)

        self.subs = [sub_image, sub_stamped_roi]
        if self.approximate_sync:
            slop = rospy.get_param('~slop', 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                fs=self.subs, queue_size=queue_size, slop=slop)
        else:
            sync = message_filters.TimeSynchronizer(
                fs=self.subs, queue_size=queue_size)

        sync.registerCallback(self.callback)


    def callback(self, image_msg, stamped_roi_msg):
        try:
            # transform image to RGB, float, CHW
            img = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="rgb8")
            img = np.asarray(img, dtype=np.float32)
            img = img.transpose(2, 0, 1)  # (H, W, C) -> (C, H, W)
        except Exception as e:
            rospy.logerr("Failed to convert image: %s" % str(e))
            return

        if self.gpu >= 0:
            chainer.cuda.get_device_from_id(self.gpu).use()

        stamped_roi = StampedRoi()
        for i, roi in enumerate(stamped_roi_msg.roi_array):
            signal = stamped_roi_msg.signals[i]

            croped_img = img[:,
                             roi.y_offset : roi.y_offset + roi.height,
                             roi.x_offset : roi.x_offset + roi.width]

            bboxes, labels, scores = self.model.predict([croped_img])
            bboxes, labels, scores = bboxes[0], labels[0], scores[0]

            # get highest score bbox
            bbox = bboxes[np.argmax(scores)]

            stamped_roi.roi_array.append(
                RegionOfInterest(
                    x_offset = bbox[1],
                    y_offset = bbox[0],
                    width = bbox[3] - bbox[1],
                    height = bbox[2] - bbox[0]))
            stamped_roi.signals.append(signal)

        stamped_roi.header = stamped_roi_msg.header
        self.stamped_roi_pub.publish(stamped_roi)


if __name__ == '__main__':
    rospy.init_node("tlr_ssd_detector")
    ssd = TLRSSDDetector()
    rospy.spin()

#!/usr/bin/env python
# -*- coding: utf-8 -*-

from cv_bridge import CvBridge
import numpy as np
import rospy

from sensor_msgs.msg import Image
from autoware_msgs.msg import StampedRoi
import message_filters

import chainer
from chainercv.links import SSD300
from chainercv.links import SSD512
from chainercv.visualizations import vis_bbox


class SSDObjectDetector():

    def __init__(self):
        super(SSDObjectDetector, self).__init__()
        self.gpu = rospy.get_param("~gpu", -1)
        self.cv_bridge = CvBridge()

        # load model
        self.label_names = ['trafficlight']
        rospy.loginfo("Loaded %d labels" % len(self.label_names))

        model_path = rospy.get_param("~model_path", None)
        model_name = rospy.get_param('~model', 'ssd300')
        if model_name == 'ssd300':
            model_class = SSD300
        elif model_name == 'ssd512':
            model_class = SSD512
        else:
            rospy.logerr('Unsupported ~model: {}'.format(model_name))

        self.model = model_class(n_fg_class=len(self.label_names),
                                 pretrained_model=model_path)

        if self.gpu >= 0:
            chainer.cuda.get_device_from_id(self.gpu).use()
            self.model.to_gpu()

        rospy.loginfo("Loaded model: %s" % model_path)

        self.stamped_roi_pub = rospy.Publisher(
            '~output_stamped_roi', StampedRoi, queue_size=1)

        sub_image = message_filters.Subscriber(
            '~input_image', Image, queue_size=1, buff_size=2**24)
        sub_stamped_roi = message_filters.Subscriber(
            '~input_stamp_roi', StampedRoi, queue_size=1, buff_size=2**24)
        self.subs = [sub_image, sub_stamped_roi]
        if rospy.get_param('~approximate_sync', True):
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
        for projcted_roi in stamped_roi_msg.roi_array:

            croped_img = img[projected_roi.y_offset:
                             projected_roi.y_offset + projected_roi.height,
                             projected_roi.x_offset:
                             projected_roi.x_offset + projected_roi.width]

            bboxes, labels, scores = self.model.predict([croped_img])
            bboxes, labels, scores = bboxes[0], labels[0], scores[0]

            stamped_roi = StampedRoi()
            prev_score = 0
            for bbox, scores in zip(bboxes, scores):
                if scores > prev_scores:
                    best_match_box = bbox
                    prev_scoers = scores

            roi = RegionOfInterest(x_offset = best_match_bbox[1],
                                   y_offset = best_match_bbox[0],
                                   width = best_match_bbox[3] - best_match_bbox[1],
                                   height = best_match_bbox[2] - best_match_bbox[0])
            stamped_roi.roi_array.append(roi)

        self.stamped_roi_pub.publish(stamped_roi)


if __name__ == '__main__':
    rospy.init_node("ssd_object_detector")
    ssd = SSDObjectDetector()
    rospy.spin()

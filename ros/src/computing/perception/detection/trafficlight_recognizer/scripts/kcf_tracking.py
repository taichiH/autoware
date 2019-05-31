import cv2
import numpy as np

import rospy
import message_filters
import cv_bridge

from sensor_msgs.msg import Image
from autoware_msgs.msg import DetectedObjectArray
from jsk_recognition_msgs.msg import Int32Stamped

class KcfTracking():

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.tracker = cv2.TrackerKCF_create()
        # self.tracker = cv2.TrackerTLD_create()
        self.prev_id = 0
        self.missing_box = False

        queue_size = rospy.get_param('~queue_size', 100)
        sub_nearest_image = message_filters.Subscriber(
            '/feat_proj_bbox/nearest_box', Image, queue_size=queue_size)
        sub_box = message_filters.Subscriber(
            '/detection/image_detector/objects', DetectedObjectArray, queue_size=queue_size)
        sub_signal_id = message_filters.Subscriber(
            '/feat_proj_bbox/signal_id', Int32Stamped, queue_size=queue_size)
        self.subs = [sub_nearest_image, sub_box, sub_signal_id]

        if rospy.get_param('~approximate_sync', False):
            slop = rospy.get_param('~slop', 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                fs=self.subs, queue_size=queue_size, slop=slop)
        else:
            sync = message_filters.TimeSynchronizer(
                fs=self.subs, queue_size=queue_size)
            sync.registerCallback(self.callback)

    def callback(self, imgmsg, boxmsg, idmsg):
        br = self.bridge
        image = br.imgmsg_to_cv2(imgmsg, desired_encoding='bgr8')

        # init tracker when change looking traffic light
        if self.prev_id != idmsg.data or self.missing_box:
            # get nearest box from center
            init_box = None
            min_distance = 24 ** 24
            print(image.shape)
            center = np.array([image.shape[1] * 0.5, image.shape[0]])
            for obj in boxmsg.objects:
                box = (obj.x, obj.y, obj.width, obj.height)
                distance = np.linalg.norm(np.array([obj.x, obj.y]) - center)
                if distance < min_distance:
                    init_box = box
                    min_distance = distance
                    self.tracker.init(image, init_box)
                    self.prev_id = idmsg.data

        # update tracker
        track, bbox = self.tracker.update(image)
        if track:
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(image, p1, p2, (0,255,0), 2, 1)
            self.missing_box = False
        else :
            cv2.putText(
                image, "Failure", (10,50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
            self.missing_box = True

        cv2.imshow("Tracking", image)
        cv2.waitKey(50)

if __name__ == '__main__':
    rospy.init_node('kcf_tracking')
    kcf_tracking = KcfTracking()
    rospy.spin()

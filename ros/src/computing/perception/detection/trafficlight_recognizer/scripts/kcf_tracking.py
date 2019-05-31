import cv2
import numpy as np

import rospy
import message_filters
import cv_bridge

from sensor_msgs.msg import Image
from autoware_msgs.msg import DetectedObjectArray
from autoware_msgs.msg import ImageRect

class KcfTracking():

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.tracker = cv2.TrackerKCF_create()
        # self.tracker = cv2.TrackerTLD_create()
        self.prev_id = 0
        self.rect_flag = False
        self.missing_box = False
        self.nearest_info = ImageRect()
        self.prev_box = None

        self.image_pub = rospy.Publisher(
            '/kcf_tracking/output', Image, queue_size=1)

        rospy.Subscriber(
            '/feat_proj_bbox/nearest_info', ImageRect, self.image_rect_callback)

        queue_size = rospy.get_param('~queue_size', 100)
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

    def image_rect_callback(self, msg):
        self.rect_flag = True
        self.nearest_info = msg

    def callback(self, imgmsg, boxmsg):
        if not self.rect_flag:
            return

        br = self.bridge
        image = br.imgmsg_to_cv2(imgmsg, desired_encoding='bgr8')

        roi_x = self.nearest_info.x
        roi_y = self.nearest_info.y
        roi_width = self.nearest_info.width
        roi_height = self.nearest_info.height
        signal_id = self.nearest_info.score

        # init tracker when change looking traffic light
        # if self.prev_id != signal_id or self.missing_box:
        if self.prev_id != signal_id or self.missing_box:
            # get nearest box from center
            box_on_roi = self.prev_box
            min_distance = 24 ** 24
            center = np.array([image.shape[1] * 0.5, image.shape[0]])

            if len(boxmsg.objects) > 0:
                for obj in boxmsg.objects:
                    box = (obj.x, obj.y, obj.width, obj.height)
                    distance = np.linalg.norm(np.array([obj.x, obj.y]) - center)
                    if distance < min_distance:
                        box_on_roi = box
                        min_distance = distance
                        self.prev_box = box_on_roi

            x = roi_x + box_on_roi[0]
            y = roi_y + box_on_roi[1]
            w = box_on_roi[2]
            h = box_on_roi[3]
            init_box = (x, y, w, h)
            p1 = (int(init_box[0]), int(init_box[1]))
            p2 = (int(init_box[0] + init_box[2]), int(init_box[1] + init_box[3]))
            cv2.rectangle(image, p1, p2, (255,0,0), 2, 1)

            self.tracker.init(image, init_box)
            self.prev_id = signal_id

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

        vis_msg = br.cv2_to_imgmsg(image, encoding='bgr8')
        self.image_pub.publish(vis_msg)

if __name__ == '__main__':
    rospy.init_node('kcf_tracking')
    kcf_tracking = KcfTracking()
    rospy.spin()

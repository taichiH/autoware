#!/usr/bin/env python

import cv2
import numpy as np

import rospy
import cv_bridge
from sensor_msgs.msg import Image
from autoware_msgs.msg import DetectedObject, DetectedObjectArray

class ImageArrayToImage():

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.width = 1920
        self.height = 1200
        self.image_pub = rospy.Publisher(
            '/image_array_to_image/output', Image, queue_size=1)
        rospy.Subscriber(
            '/feat_proj_bbox/output/roi_image_array', DetectedObjectArray, self.callback)

    def callback(self, objects):
        bridge = self.bridge
        image = np.zeros((self.height, self.width, 3), np.uint8) # y, x, ch
        for obj in objects.objects:
            img = bridge.imgmsg_to_cv2(obj.roi_image, desired_encoding='bgr8')
            y = obj.y
            x = obj.x
            print(y, x)
            image[y:y+img.shape[0], x:x+img.shape[1]] = img

        vis_msg = bridge.cv2_to_imgmsg(image, encoding='bgr8')
        self.image_pub.publish(vis_msg)

if __name__=='__main__':
    rospy.init_node('image_array_to_image')
    image_array_to_image = ImageArrayToImage()
    rospy.spin()

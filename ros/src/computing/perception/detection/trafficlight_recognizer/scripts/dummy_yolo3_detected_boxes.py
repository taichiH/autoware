#!/usr/bin/env python

from autoware_msgs.msg import DetectedObject, DetectedObjectArray
import rospy

class DummyYolo3DetectedBoxes(object):

    def __init__(self):
        self.seq = 0
        self.frame_id = "base_link"
        self.pub = rospy.Publisher('~output', DetectedObjectArray, queue_size=1)

        rate = rospy.get_param('~rate', 1)
        self.timer = rospy.Timer(rospy.Duration(1. / rate), self.publish)

    def publish(self, event):
        detected_object_array = DetectedObjectArray()
        detected_object_array.header.seq = self.seq
        detected_object_array.header.frame_id = self.frame_id
        detected_object_array.header.stamp = event.current_real

        for i in range(1):
            detected_object = DetectedObject()
            detected_object.x = 136 # on croped_roi_image
            detected_object.y = 85 # on croped_roi_image
            detected_object.width = 40
            detected_object.height = 15
            detected_object_array.objects.append(detected_object)
        self.pub.publish(detected_object_array)

if __name__ == '__main__':
    rospy.init_node('dummy_yolo3_detected_boxes')
    DummyYolo3DetectedBoxes()
    rospy.spin()

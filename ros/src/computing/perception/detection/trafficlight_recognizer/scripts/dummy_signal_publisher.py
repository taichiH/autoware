#!/usr/bin/env python

from autoware_msgs.msg import Signals
from autoware_msgs.msg import ExtractedPosition
from jsk_topic_tools.srv import ChangeTopic
import rospy

class DummySignalePublisher(object):

    def __init__(self):
        self.seq = 0
        self.frame_id = "base_link"
        self.dynamic_signal = 1
        self.pub = rospy.Publisher('~output', Signals, queue_size=1)
        rospy.Service('~change_signal', ChangeTopic, self.service_callback);

        rate = rospy.get_param('~rate', 1)
        self.timer = rospy.Timer(rospy.Duration(1. / rate), self.publish)

    def service_callback(self, req):
        try:
            self.dynamic_signal = int(req.topic)
        except:
            rospy.logerr('input integer value string')
            return

    def publish(self, event):
        signals_msg = Signals()
        signals_msg.header.seq = self.seq
        signals_msg.header.frame_id = self.frame_id
        signals_msg.header.stamp = event.current_real

        for i in range(1):
            signal = ExtractedPosition()
            signal.u = 406
            signal.v = 138
            signal.z = 30.0
            signal.signalId = self.dynamic_signal
            signals_msg.Signals.append(signal)
        self.pub.publish(signals_msg)

if __name__ == '__main__':
    rospy.init_node('dummy_signal_publisher')
    DummySignalePublisher()
    rospy.spin()

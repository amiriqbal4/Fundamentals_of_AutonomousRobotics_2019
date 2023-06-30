#!/usr/bin/env python
import rospy
from random import *
from std_msgs.msg import String
from duckietown_msgs.msg import LanePose

def test_duck():
    pub = rospy.Publisher('/PID_node/lane_pose', LanePose, queue_size=20)
    rospy.init_node('test_duck', anonymous = True)
    rate=rospy.Rate(10) #2Hz
    while not rospy.is_shutdown():
        msg = LanePose()
	msg.d = uniform(-0.02, 0.02)
	msg.phi = uniform(-0.25, .25)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        test_duck()
    except rospy.ROSInterruptException:
        pass

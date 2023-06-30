#!/usr/bin/env python
import rospy
from random import *
from std_msgs.msg import String
from rogerleo_hw2.msg import id_and_point

def node_1():
    pub = rospy.Publisher('obstacles_detected', id_and_point, queue_size=20)
    rospy.init_node('node_1', anonymous = True)
    rate=rospy.Rate(2) #2Hz
    while not rospy.is_shutdown():
        msg = id_and_point()
        #msg.points = [ uniform(1, 20), uniform(1, 10), uniform(1, 10)]
	msg.id = "Duck detected at -"
        #from random import *,# randint(1, 100)
	msg.points.x = uniform(1, 20)
       	msg.points.y = uniform(1, 20)
	msg.points.z = uniform(1, 20)
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        node_1()
    except rospy.ROSInterruptException:
        pass


#!/usr/bin/env python
import rospy
from random import *
from std_msgs.msg import String
from rogerleo_hw2.msg import id_and_point 


def callback(data):
    #def __init__(self):
    #rospy.loginfo(str(data.id) + " at position")
    #rospy.loginfo( "  x = " +str(data.points.x))
    #rospy.loginfo( " y = " +str(data.points.y))
    #rospy.loginfo( " z = " +str(data.points.z))
    print(str(data.id) + " at position")
    print( " x = " +str(data.points.x))
    print( " y = " +str(data.points.y))
    print( " z = " +str(data.points.z))

    #rospy.get_caller_id() +
def node_3():
    rospy.init_node('node_3', anonymous=True)
    rospy.Subscriber("registered_obstacles", id_and_point, callback)

    rospy.spin()

if __name__ == '__main__':
    node_3()

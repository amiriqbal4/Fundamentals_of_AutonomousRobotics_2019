#!/usr/bin/env python
import rospy
from random import *
from std_msgs.msg import String
from rogerleo_hw2.msg import id_and_point


xo = 1.0
yo = 1.0
zo = 1.0
id = 0

#class HW2_AMIR:

def callback(data):
    global xo, yo, zo, id
    #def __init__(self):
    rospy.loginfo( "I have subscribed to node_1")
    #rospy.loginfo( " y = " +str(data.points.y))
    #rospy.loginfo( " z = " +str(data.points.z))
    id += 1
    xo=data.points.x
    yo=data.points.y
    zo=data.points.z

def node_2_sub():
    global xo, yo, zo, id
    rospy.init_node('node_2_sub', anonymous=True)
    rospy.Subscriber("obstacles_detected", id_and_point, callback)
    pub2 = rospy.Publisher('registered_obstacles', id_and_point, queue_size=20)
    #rospy.spin()
    rate=rospy.Rate(2) #2Hz
    while not rospy.is_shutdown():
        global xo, yo, zo,id
        msg = id_and_point()
        #msg.points = [ uniform(1, 20), uniform(1, 10), uniform(1, 10)]
        msg.id = "Detected obstacle id = " +str(id) 
        #from random import *,# randint(1, 100)
        msg.points.x = xo
        msg.points.y = yo
        msg.points.z = zo
        pub2.publish(msg)
        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    node_2_sub()

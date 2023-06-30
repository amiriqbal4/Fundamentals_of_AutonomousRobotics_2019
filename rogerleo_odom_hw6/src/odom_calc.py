#!/usr/bin/env python
from math import sin,cos,pi
import rospy
from geometry_msgs.msg import Vector3
import matplotlib.pyplot as plt


class Odom_Calc:
    def __init__(self):
        self.x = 0.30
        self.y = 0.0
        self.theta = 0.0
        self.x_list = list()
        self.y_list = list()

    def ticks_cb(self, msg):
        l = msg.x
        r = msg.y
        delta_theta = 0.0
        # add your code to calculate odometry here
	dist_bw_wheel = 0.1 #given in the assignment (=2L)
	del_s = (l+r)/2
	#del_d = del_s
	del_th = (r-l)/dist_bw_wheel
	del_x = del_s*cos(self.theta + del_th/2)
	del_y = del_s*sin(self.theta + del_th/2)
	self.theta += del_th
	self.x += del_x
	self.y += del_y
        # make sure to update self.x, self.y, self.theta
        if self.theta > pi:
            self.theta -= 2*pi
        if self.theta < -pi:
            self.theta += 2*pi
        self.x_list.append(self.x)
        self.y_list.append(self.y)

        

if __name__ == '__main__':
    try:
        rospy.init_node('odom_calc', anonymous=True)
        oc = Odom_Calc()
        rospy.Subscriber("wheel_tick", Vector3, oc.ticks_cb)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():     
            plt.plot(oc.x_list, oc.y_list,'ro-',)
            plt.axis([-0.20,1.2,-1,1])
            plt.xlabel('x (m)')
            plt.ylabel('y (m)')
            plt.title('Vehicle Odometry (Starting Point = (0.3, 0))')
            plt.pause(0.05)        
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

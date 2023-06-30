#!/usr/bin/env python

import rospy
from duckietown_msgs.msg import Twist2DStamped

if __name__ == '__main__':
    rospy.init_node('open_loop', anonymous=True)
    pub = rospy.Publisher('~car_cmd', Twist2DStamped, queue_size=1)
    rate = rospy.Rate(10) # 10hz
    t_start = rospy.get_time()
    while not rospy.is_shutdown():
        t = rospy.get_time()
        msg = Twist2DStamped()
        dt = t - t_start
        if dt > 5 and dt < 8:
            msg.v = 0.5
            msg.omega = 0.0        
        elif dt > 9 and dt <11 :
            msg.v = 0.55
            msg.omega = 1.57        
	
	elif dt > 12 and dt < 15:
            msg.v = 0.5
            msg.omega = 0.0       

	elif dt > 16 and dt < 18:
            msg.v = 0.55
            msg.omega = 1.57        


        else:
            msg.v = 0
            msg.omega = 0
        pub.publish(msg)
        rate.sleep()

### NOTE  ###

# 1.  The code I used to draw the pattern is given below. It is different than the ideal world (no slip, perfect execution) code.
# 2.  I re caliberated wheels and used it for open loop control
# 3.  I intentionally added angular velocity in straight path to compensate for slopes
# 4.  The commanded turn angle (omega*t) was intentionally kept greater than desired value to compensate for the slip and friction

#### The code that worked ####

#   if dt > 5 and dt < 8:
#            msg.v = 0.5
#            msg.omega = -0.34  #small omega to compesate slope
#        elif dt > 9 and dt <12 :
#            msg.v = 0.55
#            msg.omega = 2.4  #More than ideal omega to compensate slip and imperfect execution

#        elif dt > 13 and dt < 16:
#            msg.v = 0.5
#            msg.omega = -0.30 #small omega to compesate slope

#        elif dt > 17 and dt < 20:
#            msg.v = 0.55
#            msg.omega = 2.0  #More than ideal omega to compensate slip and imperfect execution




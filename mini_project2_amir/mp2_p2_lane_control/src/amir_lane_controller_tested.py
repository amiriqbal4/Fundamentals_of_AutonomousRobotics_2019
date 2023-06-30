#!/usr/bin/env python
import math
import time
import numpy as np
import rospy
from duckietown_msgs.msg import Twist2DStamped, LanePose #WheelsCmdStamped, BoolStamped,

class PID_lane_controller():
        def __init__(self):
                self.node_name = rospy.get_name()
# Controller parameters are set based on intuition. Needs to be tuned later on.         
                self.k_P_d = 8.5
                self.k_P_phi = 6.50
                self.k_D_d = 0.01
                self.k_D_phi = 0.02
                self.k_I_d = 02.50
                self.k_I_phi = 0.2

                #self.d_error = 0
                #self.phi_error = 0
                self.d_integral_s = 0
                self.phi_integral_s = 0
                self.prev_d_error = 0
                self.prev_phi_error = 0
                self.v_ref = 0.5
                self.omega_ref = 0.0
                self.v_max = 1
                self.omega_max = 10.0

                self.d_ref = -0.05 # d_ref = 0 for driving in center d_ref = +/- 0.11 for staying at left/right lane edge.

                self.phi_ref = 0 #for all cases
                self.t_start = None

# Publisher and subscriber
                self.pub_control= rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
                self.sub_lane_pose = rospy.Subscriber("~lane_pose", LanePose, self.PID_Implementation, queue_size=1)
                #/<duckiebot>/lane_filter_node/lane_pose
                self.rate = rospy.Rate(10) # 10hz
                #self.t_start = rospy.get_time()

                self.pose_sensed = LanePose()

        #while not rospy.is_shutdown():
        def PID_Implementation(self,pose_sensed):
                control_msg = Twist2DStamped()
                #print(pose_sensed)
                t = rospy.get_time()
                if self.t_start is not None:
                                #current_time_ms = int(round(time.time() * 1000))
                        dt = t - self.t_start
                        d_error = pose_sensed.d - self.d_ref
                        phi_error = pose_sensed.phi
                        if dt != 0:
                                d_derivative = (d_error - self.prev_d_error)/dt
                                phi_derivative = (phi_error - self.prev_phi_error)/dt
                        else:
                                d_derivative =0
                                phi_derivative=0
                        d_derivative = max(min(d_derivative,2),-2)
                        phi_derivative = max(min(phi_derivative,2),-2)


                        self.d_integral_s +=  d_error*dt
                        self.d_integral_s = max(min(self.d_integral_s,3),-3)


                        if math.fabs(d_error) < 0.01:
                                self.d_integral_s =0
                                self.phi_integral_s=0
                        d_integral = self.d_integral_s

                        self.phi_integral_s +=  phi_error*dt
                        self.phi_integral_s = max(min(self.phi_integral_s,3),-3)

                        if math.fabs(phi_error) < 0.04:
                                self.d_integral_s =0
                                self.phi_integral_s=0

                        phi_integral = self.phi_integral_s

                        control_d = self.k_P_d*d_error + self.k_D_d*d_derivative + self.k_I_d*d_integral
                        control_phi = self.k_P_phi*phi_error #+ self.k_D_phi*phi_derivative + self.k_I_phi*phi_integral
                        control = control_d + control_phi
                        control =max(min(control,4),-4)

                        control_msg.omega = -control
                        control_msg.v = 0.5/(math.fabs(control/3)+0.65)

                        self.prev_d_error = d_error
                        self.prev_phi_error = phi_error
                        #self.t_start =t
                        print ("w = ", control)
                        print ("d_error = ", d_error)
                        print ("phi_eror = ", phi_error)
                        #print ("error = ", self.prev_d_error , self.prev_phi_error)
                        print ("derivative = ", d_derivative,   phi_derivative)
                        print ("integral = ", d_integral, phi_integral)

                #print (control_msg)
                self.t_start =t
                self.pub_control.publish(control_msg)

                print(control_msg)


if __name__ == "__main__":

    rospy.init_node("PID_node", anonymous=False)  # adapted to sonjas default file

    PID_node = PID_lane_controller()

rospy.spin()




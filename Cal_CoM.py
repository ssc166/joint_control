#!/usr/bin/env python

import sympy as sp
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray, Float64

def callback(data):
    theta = data.data
    theta_1 = theta[0]
    theta_2 = theta[0] + theta[1]
    theta_3 = theta[0] + theta[1] + theta[2]
    theta_4 = theta[0] + theta[1] + theta[2] + theta[3]
    
    m1 = 2.486 + 0.3
    m2 = 1.416
    m3 = 1.739
    m4 = 3.25+12.5

    L1 = 0.171
    L2 = 0.279942
    L3 = 0.280
    L4 = 0.346

    L1c = L1 / 2
    L2c = L2 *0.75
    L3c = L3 / 2
    L4c = 0.17188
    
    x_1 = L1c * sp.cos(theta_1)
    x_2 = L1 * sp.cos(theta_1) + L2c * sp.cos(theta_2)
    x_3 = L1 * sp.cos(theta_1) + L2 * sp.cos(theta_2) + L3c*sp.cos(theta_3)
    x_4 = L1 * sp.cos(theta_1) + L2 * sp.cos(theta_2) + L3 * sp.cos(theta_3) + L4c * sp.cos(theta_4)
    x_com = (m1*x_1 + m2*x_2 + m3*x_3+ m4*x_4) / (m1 + m2 + m3 + m4)
    
    z_1c = L1c * sp.sin(theta_1)
    z_2c = L1 * sp.sin(theta_1) + L2c * sp.sin(theta_2)
    z_3c = L1 * sp.sin(theta_1) + L2 * sp.sin(theta_2) + L3c*sp.sin(theta_3)
    z_4c = L1 * sp.sin(theta_1) + L2 * sp.sin(theta_2) + L3 * sp.sin(theta_3) + L4c * sp.sin(theta_4)

    z_com = (m1*z_1c + m2*z_2c + m3*z_3c + m4*z_4c) / (m1 + m2 + m3 + m4)
    theta_P = np.arctan(float(x_com)/float(z_com))
    
    CoMlist = Float64MultiArray()
    CoMlist.data = [z_com, theta_P]

    pub.publish(CoMlist)
    
    
    
# def listener():
#     rospy.init_node('listener', anonymous=True)
#     rospy.Subscriber('array', Float64MultiArray)
#     rospy.spin()

if __name__ == '__main__':
    try:  
        while True:
            rospy.init_node('listener', anonymous=True)
            rospy.Subscriber('array', Float64MultiArray,callback)
            pub = rospy.Publisher('CoM', Float64MultiArray, queue_size=100)
            
            
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
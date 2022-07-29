#!/usr/bin/env python

import sympy as sp
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
import time


def get_deg(h):
    m2 = 1.416
    m3 = 1.739
    m4 = 3.25+12.5

    L1 = 0.171
    L2 = 0.28
    L3 = 0.28
    L4 = 0.346

    L2c = L2 *0.75
    L3c = L3 / 2

    theta_3, q2 = sp.symbols('theta_3, q2')
    
    A = (h-L1-L4)/L2
    B = m2*L2c + m3*L2+m4*L2
    C = m3*L3c + m4*L3
    
    f1 = sp.Eq(sp.cos(q2)+sp.sin(theta_3),float(A))
    f2 = sp.Eq(-float(B)*sp.sin(q2)+float(C)*sp.cos(theta_3),0)
    sol = sp.solve([f1,f2])
    solu = sol[0]

    q2 = solu[q2]
    theta_3 = solu[theta_3]
    q1 = np.pi/2
    q3 = theta_3-q1-q2
    q4 = -(q2+q3)

    # thetalistd = np.array([q1, q2, q3,q4])
    return q1, q2, q3, q4

RAD2DEG = 180/np.pi


if __name__ == '__main__':
    try:    
        
        rospy.init_node('Joint_Calculator', anonymous=True)
        pub_joint = rospy.Publisher('array', Float64MultiArray, queue_size=100)
        
        while True:
            
            print('Enter Height(m): ')
            height = input()
            t1 = time.time()
            q1, q2, q3, q4 = get_deg(float(height))
            t2 = time.time()
            
            thetalistd = Float64MultiArray()
            thetalistd.data = [q1, q2, q3,q4]
            pub_joint.publish(thetalistd)
            theta = np.array([q1, q2, q3,q4])
            print(theta)
            print(theta*RAD2DEG)
            print(t2-t1)
        
    except rospy.ROSInterruptException:
        pass
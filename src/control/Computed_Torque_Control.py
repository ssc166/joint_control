#!/usr/bin/env python

import sympy as sp
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray, Float64
from gazebo_msgs.srv import GetModelState, ApplyBodyWrench, GetLinkState
import math
import WIP_utils as utils
import modern_robotics as mr
import time

def callback(data):
    
    torque_traj = data.data
    print(torque_traj)

if __name__ == '__main__':
    try:  
        

        while True:
            rospy.init_node('CTC_Calculator', anonymous=True)
            rospy.Subscriber('Angle_Traj', Float64MultiArray,callback)
            
            
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
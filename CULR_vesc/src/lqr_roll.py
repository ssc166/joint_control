#!/usr/bin/env python

import sympy as sp
import numpy as np
import math
import time
import rospy
import os
from std_msgs.msg import Float64, Float64MultiArray
from gazebo_msgs.srv import GetModelState, GetLinkState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import *
import pylab as pl
import control
import matplotlib.pyplot as plt
import state_equation_roll as ser
import WIP_utils as utils

    
def roll_K_gain_R(A, B, Q, R):
        
    K, S, E = control.lqr(A, B, Q, R)
    
    return K, S, E


def cmg_lqr(pos_data, rps_data, imu_cmg_data):
    
    z_com = 0.5004170132265042
    A, B, C, D = ser.Cal_Roll_SS(z_com)

    Q = sp.Matrix([ [1,    0,    0,    0],
                    [0,    10,    0,    0],
                    [0,    0,    1,    0],
                    [0,    0,    0,    1]])

    R = sp.Matrix([ [1] ])

    K, S, E = roll_K_gain_R(A, B, Q, R)
    print('K: ', K)
    
    x0 = np.array([imu_cmg_data[0], pos_data, imu_cmg_data[2], rps_data])
    x_next = (A-B*K)@x0
    
    return x_next
    

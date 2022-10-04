#!/usr/bin/env python

# from this import d
import sympy as sp
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray, Float64
from gazebo_msgs.srv import GetModelState, ApplyBodyWrench, GetLinkState
from sensor_msgs.msg import Imu, JointState
import math
import WIP_utils as utils
import time
import os
import Cal_joint as cj
import WIP_utils as utils
import control
import dmpc_tool_pitch as mpc
import matplotlib.pyplot as plt
from sympy.physics.mechanics import *
import sys
    
RAD2DEG = 180/np.pi

def pitch_mpc_init(theta_1, dtheta_1, wheel_rps_data, ankle_pos_data, ankle_rps_data):
    theta_2 = theta_1 + ankle_pos_data[0][0]
    dtheta_2 = dtheta_1 + ankle_rps_data[0][0]
    
    x0 = np.array([theta_1, theta_2,wheel_rps_data[0][0], dtheta_1, dtheta_2])
    mpc_model, estimator, u0 = mpc.set_model(x0)
    u = mpc_model.make_step(x0)
    
    return mpc_model, estimator, u

def pitch_mpc(theta_1, dtheta_1, wheel_rps_data, ankle_pos_data, ankle_rps_data, mpc_model, estimator):
    theta_2 = theta_1 + ankle_pos_data[0][0]
    dtheta_2 = dtheta_1 + ankle_rps_data[0][0]
    
    y_step = np.array([theta_1, theta_2,wheel_rps_data[0][0], dtheta_1, dtheta_2])
    x0 = estimator.make_step(y_step)
    u = mpc_model.make_step(x0)
    
    return u

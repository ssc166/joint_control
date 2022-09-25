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
import modern_robotics as mr
import time
import os
import Cal_joint as cj
import WIP_utils as utils
import control
import dmpc_tool_pitch as mpc
import matplotlib.pyplot as plt
from sympy.physics.mechanics import *
import sys

def pitch_control_init(pitch_pos_state, pitch_vel_state):
    x0 = np.array([pitch_pos_state[1], pitch_pos_state[2],pitch_vel_state[0],pitch_vel_state[1],pitch_vel_state[2]])
    mpc_model, estimator, u0 = mpc.set_model(x0)
    u = mpc_model.make_step(x0)

    return mpc_model,estimator, u

def pitch_control(pitch_pos_state, pitch_vel_state, mpc_model,estimator):
    y_step = np.array([pitch_pos_state[1], pitch_pos_state[2],pitch_vel_state[0],pitch_vel_state[1],pitch_vel_state[2]])
    x0 = estimator.make_step(y_step)
    u = mpc_model.make_step(x0)

    return u

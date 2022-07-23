#!/usr/bin/env python

import sympy as sp
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray, Float64
from gazebo_msgs.srv import GetModelState, ApplyBodyWrench, GetLinkState
import math
import WIP_utils as utils

def get_link_state(link_name_main, reference_frame):
    
    get_state = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
    
    link_state = get_state(link_name= link_name_main, reference_frame = reference_frame)

    return link_state

def get_link_ori(link_name_main, reference_frame):
    global link_state
    
    link_state = get_link_state(link_name_main, reference_frame)
    
    X, Y, Z = utils.quaternion_to_euler_angle(link_state.link_state.pose.orientation)
    link_ori_x = X
    link_ori_y = Y
    link_ori_z = Z
     
    return link_ori_x, link_ori_y, link_ori_z

def get_cur_deg():

    ankle_link = 'ankle_link'
    knee_ankle_link = 'knee_ankle_link'
    hip_to_knee_link = 'hip_to_knee_link'
    cmg_link = 'cmg'
    link_name_list = [ankle_link, knee_ankle_link, hip_to_knee_link, cmg_link]

    L1_x, L1_y, L1_z = get_link_ori(link_name_list[0], 'world')
    L2_x, L2_y, L2_z = get_link_ori(link_name_list[1], link_name_list[0])
    L3_x, L3_y, L3_z = get_link_ori(link_name_list[2], link_name_list[1])
    L4_x, L4_y, L4_z = get_link_ori(link_name_list[3], link_name_list[2])

    thetalist = np.array([L1_y, L2_y, L3_y, L4_y])

    return thetalist

def Cal_Traj():

    thetalist = get_cur_deg()

    time, traj = utils.Trapezoidal_Traj_Gen_Given_Amax_and_T(1.5,2,0.01)
    traj_th1 = utils.Path_Gen(thetalist[0], theta[0], traj[:,0])
    traj_th2 = utils.Path_Gen(thetalist[1], theta[1], traj[:,0])
    traj_th3 = utils.Path_Gen(thetalist[2], theta[2], traj[:,0])
    traj_th4 = utils.Path_Gen(thetalist[3], theta[3], traj[:,0])

    n = len(time)

    return n, traj_th1, traj_th2, traj_th3, traj_th4

def callback(data):
    global theta
    theta = data.data
    theta_1 = theta[0]
    theta_2 = theta[0] + theta[1]
    theta_3 = theta[0] + theta[1] + theta[2]
    theta_4 = theta[0] + theta[1] + theta[2] + theta[3]

    n, traj_th1, traj_th2, traj_th3, traj_th4 = Cal_Traj()
    
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

    pub_roll.publish(CoMlist)
    pub_pitch.publish(CoMlist)

#######################################################################################################

if __name__ == '__main__':
    try:  
        while True:
            rospy.init_node('listener', anonymous=True)
            rospy.Subscriber('array', Float64MultiArray,callback)
            pub_roll = rospy.Publisher('CoM_Roll', Float64MultiArray, queue_size=100)
            pub_pitch = rospy.Publisher('CoM_Pitch', Float64MultiArray, queue_size=100)
            
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
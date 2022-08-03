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
import os
import init_traj as it


def gazebo_setting():
    os.system('rosservice call /gazebo/set_physics_properties "{time_step: 0.001, max_update_rate: 1000.0, gravity: {x: 0.0, y: 0.0, z: -9.8}, ode_config: {auto_disable_bodies: False, sor_pgs_precon_iters: 0, sor_pgs_iters: 200, sor_pgs_w: 1.0, sor_pgs_rms_error_tol: 0.0, contact_surface_layer: 0.001, contact_max_correcting_vel: 100.0, cfm: 0.0, erp: 0.2, max_contacts: 20}}"')
    # os.system('rosservice call gazebo/unpause_physics')
    rospy.loginfo("Simulation Start")

def get_link_state(link_name_main, reference_frame):
    
    get_state = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
    
    link_state = get_state(link_name= link_name_main, reference_frame = reference_frame)

    return link_state

def get_link_ori(link_name_main, reference_frame):
    
    link_state = get_link_state(link_name_main, reference_frame)
    
    X, Y, Z = utils.quaternion_to_euler_angle(link_state.link_state.pose.orientation)
    link_ori_x = X
    link_ori_y = Y
    link_ori_z = Z
     
    return link_ori_x, link_ori_y, link_ori_z

def get_link_vel(link_name_main, reference_frame):
    
    link_state = get_link_state(link_name_main, reference_frame)
    
    link_vel_x = link_state.link_state.twist.angular.x
    link_vel_y = link_state.link_state.twist.angular.y
    link_vel_z = link_state.link_state.twist.angular.z
    
    return link_vel_x, link_vel_y, link_vel_z

def get_cur_deg():
    global link_name_list

    ankle_link = 'ankle_link'
    knee_ankle_link = 'knee_ankle_link'
    hip_to_knee_link = 'hip_to_knee_link'
    cmg_link = 'cmg'
    link_name_list = [ankle_link, knee_ankle_link, hip_to_knee_link, cmg_link]

    L1_x, L1_y, L1_z = get_link_ori(link_name_list[0], 'world')
    L2_x, L2_y, L2_z = get_link_ori(link_name_list[1], link_name_list[0])
    # L3_x, L3_y, L3_z = get_link_ori(link_name_list[2], 'world')
    L4_x, L4_y, L4_z = get_link_ori(link_name_list[3], link_name_list[2])

    L3_y = -(L2_y + L4_y)

    thetalist = np.array([L1_y, L2_y, L3_y, L4_y])
 
    return thetalist

def get_cur_vel():

    L1_x, L1_y, L1_z = get_link_vel(link_name_list[0], 'world')
    L2_x, L2_y, L2_z = get_link_vel(link_name_list[1], 'world')
    L3_x, L3_y, L3_z = get_link_vel(link_name_list[2], 'world')
    L4_x, L4_y, L4_z = get_link_vel(link_name_list[3], 'world')

    dthetalist = np.array([L1_y, L2_y, L3_y, L4_y])
 
    return dthetalist  

def get_deg():

    time, traj = utils.Trapezoidal_Traj_Gen_Given_Amax_and_T(1,2,0.03)
    height_path = utils.Path_Gen(1.077, 0.9, traj[:,0])

    m2 = 1.416
    m3 = 1.739
    m4 = 16.09

    L1 = 0.171
    L2 = 0.28
    L3 = 0.28
    L4 = 0.346

    L2c = L2 - 0.045289
    L3c = L3 - 0.18878

    q1list = np.array([0])
    q2list = np.array([0])
    q3list = np.array([0])
    q4list = np.array([0])

    n = len(time)
    for i in np.arange(0,n-1):

        
        A = (height_path[i]-L1-L4)/L2
        B = m2*L2c + m3*L2+m4*L2
        C = m3*L3c + m4*L3

        theta_3, q2 = sp.symbols('theta_3, q2')
        f1 = sp.Eq(sp.cos(q2)+sp.sin(theta_3),float(A))
        f2 = sp.Eq(-float(B)*sp.sin(q2)+float(C)*sp.cos(theta_3),0)

        sol = sp.solve([f1,f2])


        if height_path[i] < 1.077:
            solu = sol[1]
        else:
            solu = sol[0]
        q2 = solu[q2]
        theta_3 = solu[theta_3]        
        q1 = np.pi/2
        q3 = theta_3-q1-q2
        q4 = -(q2+q3)

        q1list = np.vstack((q1list,q1))
        q2list= np.vstack((q2list,q2))
        q3list= np.vstack((q3list,q3))
        q4list= np.vstack((q4list,q4))


    return q1list, q2list, q3list, q4list

def init_pos():
    # rospy.loginfo("Start Calculation")
    q2, q3, q4 = it.joint_traj()
    # rospy.loginfo("Calculate Joint Trajectory")
    rate = rospy.Rate(1000)

    for i in range(0,len(q2)-1):
        pub_2.publish(q2[i])
        pub_3.publish(q3[i])
        pub_4.publish(q4[i])

        rate.sleep()
    rospy.loginfo("Complete Initialization")

# def callback(data):
    
#     torque_traj = data.data
#     print(torque_traj)

#######################################################################################################


m1 = 2.486
m2 = 1.416
m3 = 1.739
m4 = 16.09

L1 = 0.171
L2 = 0.28
L3 = 0.280
L4 = 0.346


#######################################################################################################

if __name__ == '__main__':
    try:  
        rospy.init_node('CTC_Calculator', anonymous=True)
        pub_2 = rospy.Publisher('/wheeled_inverted_pendulum/ankle_pitch/command', Float64, queue_size=100)
        pub_3 = rospy.Publisher('/wheeled_inverted_pendulum/knee/command', Float64, queue_size=100)
        pub_4 = rospy.Publisher('/wheeled_inverted_pendulum/hip_pitch/command', Float64, queue_size=100)
        rate = rospy.Rate(1000)
        gazebo_setting()
        RAD2DEG = 180/np.pi

        q1d = 0
        q2d = -0.745239
        q3d = 1.6307
        q4d = -0.885463

        thetalistd = np.array([0, q2d, q3d, q4d])

        init_pos()
        cur_time = time.time()    
        sec_time = time.time() 

        while True:
            # rospy.Subscriber('Angle_Traj', Float64MultiArray,callback)
            last_time = cur_time
            cur_time = time.time()
            # sec_cur_time = time.time()
            dt = cur_time - last_time 
            # sec =  sec_cur_time - sec_time

            # thetalist = get_cur_deg()
            # dthetalist = get_cur_vel()

            print("thetalistd: ", thetalistd)
            # print("thetalist: ", thetalist)
            # print("dthetalist: ", dthetalist)
            print("dt: ", dt)
            print('----------------------------------')

            # pub_2.publish(thetalistd[1])
            # pub_3.publish(thetalistd[2])
            # pub_4.publish(thetalistd[3])
            
            
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
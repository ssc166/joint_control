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

def CTC(thetalist, dthetalist, thetalistd, dt, Mlist, Slist):

    dtheta1d = (thetalistd[0] - thetalist[0])/dt
    dtheta2d = (thetalistd[1] - thetalist[1])/dt
    dtheta3d = (thetalistd[2] - thetalist[2])/dt
    dtheta4d = (thetalistd[3] - thetalist[3])/dt
    dthetalistd = np.array([dtheta1d, dtheta2d, dtheta3d, dtheta4d])

    ddtheta1d = (dthetalistd[0] - dthetalist[0])/dt
    ddtheta2d = (dthetalistd[1] - dthetalist[1])/dt
    ddtheta3d = (dthetalistd[2] - dthetalist[2])/dt
    ddtheta4d = (dthetalistd[3] - dthetalist[3])/dt
    ddthetalistd = np.array([ddtheta1d, ddtheta2d, ddtheta3d, ddtheta4d])

    

    torque = mr.ComputedTorque(thetalist, dthetalist, eint, g, Mlist, Glist, Slist, \
                   thetalistd, dthetalistd, ddthetalistd, Kp, Ki, Kd)

    return torque


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

eint = np.array([0.2, 0.2, 0.2, 0.2])
g = np.array([0, 0, -9.8])

G1 = np.diag([0.010267, 0.010267, 0.00666, m1, m1, m1]) #X
G2 = np.diag([0.010192583, 0.002390605, 0.009599672, m2, m2, m2])
G3 = np.diag([0.008595913, 0.002944951, 0.007917693, m3, m3, m3])
G4 = np.diag([0.2715612, 0.123573848, 0.20661847, m4, m4, m4])

Glist = np.array([G1, G2, G3, G4])

Kp = 1.3
Ki = 0.1
Kd = 1.0

def get_Mlist_Slist(thetalist):

    theta_1 = thetalist[0]
    theta_2 = thetalist[0] + thetalist[1]
    theta_3 = thetalist[0] + thetalist[1] + thetalist[2]
    theta_4 = thetalist[0] + thetalist[1] + thetalist[2] + thetalist[3]

    M01 = np.array([[1, 0, 0,       0],
                [0, 1, 0,       0],
                [0, 0, 1,       0],
                [0, 0, 0,       1]])
    M12 = np.array([[1, 0, 0,        0],
                    [0, 1, 0,       L1],
                    [0, 0, 1,        0],
                    [0, 0, 0,        1]])
    M23 = np.array([[ 1, 0, 0,      -L2*np.cos(theta_2)],
                    [ 0, 1, 0,      L2*np.sin(theta_2)],
                    [ 0, 0, -1,       0],
                    [ 0, 0, 0,       1]])
    M34 = np.array([[1, 0, 0,       L2*np.cos(theta_3)],
                    [0, 1, 0,       L2*np.sin(theta_3)],
                    [0, 0, -1,      0],
                    [0, 0, 0,       1]])
    M45 = np.array([[1, 0, 0,       0],
                    [0, 1, 0,      L4],
                    [0, 0, 1,       0],
                    [0, 0, 0,       1]])

    Mlist = np.array([M01, M12, M23, M34, M45])
    Slist = np.array([[0, 0, 1,      0, 0,     0],
                        [0, 0, 1,   -L1 * np.sin(theta_1), L1 * np.cos(theta_1),     0],
                        [0, 0, -1, L1*np.sin(theta_1) + L2 * np.sin(theta_2), -(L1 * np.cos(theta_1)+L2 * np.cos(theta_2)), 0],
                        [0, 0, 1, -(L1*np.sin(theta_1) + L2 * np.sin(theta_2)+L3 * np.sin(theta_3)), L1 * np.cos(theta_1) + L2 * np.cos(theta_2)-L3 * np.cos(theta_3), 0]]).T

    return Mlist, Slist

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

        ddt = 0.03

        q1d = 0
        q2d = 0.745239
        q3d = -1.6307
        q4d = 0.885463

        thetalistd = np.array([0, q2d, q3d, q4d])
        cur_time = time.time()    
        sec_time = time.time() 

        while True:
            # rospy.Subscriber('Angle_Traj', Float64MultiArray,callback)
            last_time = cur_time
            cur_time = time.time()
            # sec_cur_time = time.time()
            dt = cur_time - last_time 
            # sec =  sec_cur_time - sec_time

            thetalist = get_cur_deg()
            dthetalist = get_cur_vel()

            theta_1 = 1.5708
            theta_2 = 1.5708 + thetalist[1]
            theta_3 = 1.5708 + thetalist[1] + thetalist[2]
            theta_4 = 1.5708 + thetalist[1] + thetalist[2] + thetalist[3]
            thetal = np.array([theta_1*RAD2DEG, theta_2*RAD2DEG, theta_3*RAD2DEG, theta_4*RAD2DEG])
            Mlist, Slist = get_Mlist_Slist(thetalist)
            torquelist = CTC(thetalist, dthetalist, thetalistd, ddt, Mlist, Slist)


            print("thetalist: ", thetalist)
            print(thetal)
            print("dthetalist: ", dthetalist)
            print("dt: ", dt)
            print("torquelist: ", torquelist)
            print('----------------------------------')

            pub_2.publish(torquelist[1])
            pub_3.publish(torquelist[2])
            pub_4.publish(torquelist[3])
            
            
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
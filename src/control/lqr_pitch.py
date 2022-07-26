#!/usr/bin/env python

import sympy as sp
import numpy as np
import math
import time
import rospy
import os
from std_msgs.msg import Float64, Float64MultiArray
from gazebo_msgs.srv import GetModelState, ApplyBodyWrench, GetLinkState
from geometry_msgs.msg import *
from sensor_msgs.msg import Imu
import pylab as pl
import control
from sympy.physics.mechanics import *
from numpy.linalg import matrix_rank, eig
import matplotlib.pyplot as plt
import state_equation_pitch as sep
import WIP_utils as utils

def gazebo_setting():
    os.system('rosservice call /gazebo/set_physics_properties "{time_step: 0.001, max_update_rate: 1000.0, gravity: {x: 0.0, y: 0.0, z: -9.8}, ode_config: {auto_disable_bodies: False, sor_pgs_precon_iters: 0, sor_pgs_iters: 200, sor_pgs_w: 1.0, sor_pgs_rms_error_tol: 0.0, contact_surface_layer: 0.001, contact_max_correcting_vel: 100.0, cfm: 0.0, erp: 0.2, max_contacts: 20}}"')
    # os.system('rosservice call gazebo/unpause_physics')
    rospy.loginfo("Simulation Start")

def pitch_K_gain():
    
    K, S, E = control.lqr(A, B, Q, R)
    
    return K, S, E
        
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

def get_link_vel(link_name_main, reference_frame):
    global link_state
    
    link_state = get_link_state(link_name_main, reference_frame)
    
    link_vel_x = link_state.link_state.twist.angular.x
    link_vel_y = link_state.link_state.twist.angular.y
    link_vel_z = link_state.link_state.twist.angular.z
    
    return link_vel_x, link_vel_y, link_vel_z
    
def get_model_state(body_name):
    body_state = get_body(model_name = body_name)
    
    return body_state

def get_body_ori():
    global body_state
    
    body_state = get_model_state(body_name)
    
    X, Y, Z = utils.quaternion_to_euler_angle(body_state.pose.orientation)
    body_ori_x = X
    body_ori_y = Y
    body_ori_z = Z
    
    return body_ori_x, body_ori_y, body_ori_z

def get_body_vel():
    global body_state
    
    body_state = get_model_state(body_name)
    
    body_vel_x = body_state.twist.angular.x
    body_vel_y = body_state.twist.angular.y
    body_vel_z = body_state.twist.angular.z
    
    return body_vel_x, body_vel_y, body_vel_z

def get_wheel_state(wheel_name):
    wheel_state = get_state(link_name = wheel_name)
    
    return wheel_state
    
def get_wheel_ori():
    global wheel_state
    
    wheel_state = get_wheel_state(wheel_name)
    
    X, Y, Z = utils.quaternion_to_euler_angle(wheel_state.link_state.pose.orientation)
    wheel_ori_x = X
    wheel_ori_y = Y
    wheel_ori_z = Z
    
    return wheel_ori_x, wheel_ori_y, wheel_ori_z

def get_wheel_vel():
    global wheel_state
    
    wheel_state = get_wheel_state(wheel_name)
    
    wheel_vel_x = wheel_state.link_state.twist.angular.x
    wheel_vel_y = wheel_state.link_state.twist.angular.y
    wheel_vel_z = wheel_state.link_state.twist.angular.z
    
    return wheel_vel_x, wheel_vel_y, wheel_vel_z

def linvel2wheelvel(linvel):
    wheel_rad = 0.138/2
    wheelvel = linvel/wheel_rad
    
    return wheelvel

def get_theta_P():
    m1 = 2.486 + 0.3
    m2 = 1.416
    m3 = 1.739
    m4 = 3.25+12.5

    L1 = 0.171
    L2 = 0.28
    L3 = 0.280
    L4 = 0.346

    L1c = L1 / 2
    L2c = L2 *0.75
    L3c = L3 / 2
    L4c = 0.17188
    
    L1_x, L1_y, L1_z = get_link_ori(link_name_list[0], 'world')
    L2_x, L2_y, L2_z = get_link_ori(link_name_list[1], link_name_list[0])
    L3_x, L3_y, L3_z = get_link_ori(link_name_list[2], link_name_list[1])
    L4_x, L4_y, L4_z = get_link_ori(link_name_list[3], link_name_list[2])
    
    theta_1 = L1_y
    theta_2 = L1_y + L2_y
    theta_3 = L1_y + L2_y + L3_y
    theta_4 = L1_y + L2_y + L3_y + L4_y
    
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
    
    return theta_P

def callback(data):
    global z_com
    
    CoMlist = data.data
    z_com = CoMlist[0]

    

def print_graph():
    
    plt.plot(sec_store, deg_store)

    plt.xlabel('sec[s]')
    plt.ylabel('tilt angle[deg]')
    plt.title('Pitch tilt angle')
    plt.ylim(-2.5,2.5)
    plt.xlim(0, 15)
    # plt.legend(loc='upper right')
    plt.grid(True, axis='y')

    plt.show()
    
#######################################################################################################
A, B, C, D = sep.Cal_Pitch_SS(z_com)

# q = [phi, theta, phi_dot, theta_dot]
Q = sp.Matrix([ [0.3,    0,    0,    0],
                [0,    0.01,    0,    0],
                [0,    0,    0.1,    0],
                [0,    0,    0,    0.01]])

R = sp.Matrix([ [1] ])

K, S, E = pitch_K_gain()
ss0 = [A, B, C, D]
sys0 = control.ss(*[pl.array(mat_i).astype(float) for mat_i in ss0])
sysc = sys0.feedback(K)

RAD2DEG = 180/np.pi

get_state = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
get_body = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

wheel_state = []
wheel_name = 'wheel_link'
wheel_name_list = [wheel_name]

ankle_link = 'ankle_link'
knee_ankle_link = 'knee_ankle_link'
hip_to_knee_link = 'hip_to_knee_link'
cmg_link = 'cmg'
link_name_list = [ankle_link, knee_ankle_link, hip_to_knee_link, cmg_link]

body_state = []
body_name = 'wheeled_inverted_pendulum'
body_name_list = [body_name]

print('K: ', K)
loop_cnt = 0

deg_store = []
sec_store = []

############################################################################################
 
if __name__ == '__main__':
    try:                
        
        rospy.wait_for_service('gazebo/get_model_state')
        rospy.init_node('Pitch_Controller', anonymous=False) 
        
        pub_w = rospy.Publisher('/wheeled_inverted_pendulum/wheel/command', Float64, queue_size=100)
        rospy.Subscriber('CoM_Pitch', Float64MultiArray, callback)
        
        rate = rospy.Rate(1000)
        gazebo_setting()

        
        cur_time = time.time()    
        sec_time = time.time()  
        
        print(z_com)
                 
         
        while True:

            last_time = cur_time
            cur_time = time.time()
            sec_cur_time = time.time()
            dt = cur_time - last_time 
            sec =  sec_cur_time - sec_time
 
            wheel_ori_x, wheel_ori_y, wheel_ori_z = get_wheel_ori()
            wheel_vel_x, wheel_vel_y, wheel_vel_z = get_wheel_vel()
            
            theta_P = get_theta_P()

            x0 = np.array([wheel_ori_y,theta_P,wheel_vel_y,body_vel_y])

            u = -K @ ( x0 )
            pub_w.publish(u)

            # deg_store.append(theta_P*RAD2DEG)
            # sec_store.append(sec)
            
            if loop_cnt % 10 == 0:
                print('Wheel_velocity  (rad/s): ', wheel_vel_y)
                print('Pitch             (deg): ', theta_P*RAD2DEG)

                print('====================================')  
            
            
            loop_cnt= loop_cnt + 1

            # print('dt: ', dt)

            rate.sleep()
            
            # if loop_cnt == 1500:
            #     print_graph()
                
                

    except rospy.ROSInterruptException:
        pass

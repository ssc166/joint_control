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



def gazebo_setting():
    os.system('rosservice call /gazebo/set_physics_properties "{time_step: 0.001, max_update_rate: 1000.0, gravity: {x: 0.0, y: 0.0, z: -9.8}, ode_config: {auto_disable_bodies: False, sor_pgs_precon_iters: 0, sor_pgs_iters: 200, sor_pgs_w: 1.0, sor_pgs_rms_error_tol: 0.0, contact_surface_layer: 0.001, contact_max_correcting_vel: 100.0, cfm: 0.0, erp: 0.2, max_contacts: 20}}"')
    # os.system('rosservice call /gazebo/set_model_configuration "{model_name: wheeled_inverted_pendulum, joint_names:[hip_pitch_joint,knee_joint,ankle_pitch_joint], joint_positions: [-0.885462876712376,1.63070174355873,-0.745238866846359]}"')
    # os.system('rosservice call /gazebo/set_model_state "{model_state: {model_name: wheeled_inverted_pendulum, pose: {position: { x: 0.0, y: 0.0 ,z: -0.177142}}, reference_frame: world}}"')
    # os.system('rosservice call gazebo/unpause_physics')
    rospy.loginfo("Simulation Start")
    
def pitch_K_gain(A, B, Q, R):
    
    K, S, E = control.lqr(A, B, Q, R)

    return K, S, E

def get_link_state(link_name_main, reference_frame):
    
    get_state = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
    
    link_state = get_state(link_name= link_name_main, reference_frame = reference_frame)

    return link_state

def get_link_ori_vel(link_name_main, reference_frame):
    
    link_state = get_link_state(link_name_main, reference_frame)
    
    X, Y, Z = utils.quaternion_to_euler_angle(link_state.link_state.pose.orientation)
    link_ori_x = X
    link_ori_y = Y
    link_ori_z = Z
    
    link_vel_x = link_state.link_state.twist.angular.x
    link_vel_y = link_state.link_state.twist.angular.y
    link_vel_z = link_state.link_state.twist.angular.z
     
    return link_ori_x, link_ori_y, link_ori_z, link_vel_x, link_vel_y, link_vel_z

def get_cur_deg_vel():
    global link_name_list

    ankle_link = 'ankle'
    knee_ankle_link = 'knee_ankle_link'
    hip_to_knee_link = 'hip_to_knee_link'
    cmg_link = 'cmg'
    link_name_list = [ankle_link, knee_ankle_link, hip_to_knee_link, cmg_link]

    link1_ori_x, link1_ori_y, link1_ori_z, link1_vel_x, link1_vel_y, link1_vel_z = get_link_ori_vel(link_name_list[0], 'world')
    link2_ori_x, link2_ori_y, link2_ori_z, link2_vel_x, link2_vel_y, link2_vel_z = get_link_ori_vel(link_name_list[1], 'world')
    # link3_ori_x, link3_ori_y, link3_ori_z, link3_vel_x, link3_vel_y, link3_vel_z = get_link_ori_vel(link_name_list[2], link_name_list[1])
    # link_4ori_x, link4_ori_y, link4_ori_z, link4_vel_x, link4_vel_y, link4_vel_z = get_link_ori_vel(link_name_list[3], link_name_list[2])

    # link3_ori_y = -(link2_ori_y + link4_ori_y)
    # thetalist = np.array([[-link1_ori_y], [-link2_ori_y], [-link3_ori_y], [-link4_ori_y]])
    # dthetalist = np.array([[-link1_vel_y], [-link2_vel_y], [-link3_vel_y], [-link4_vel_y]])
 
 
    return link1_ori_y,link2_ori_y, link1_vel_y, link2_vel_y

def get_model_state(body_name):
    body_state = get_body(model_name = body_name)
    
    return body_state

def get_body_param():
    global body_state
    
    body_state = get_model_state(body_name)
    
    X, Y, Z = utils.quaternion_to_euler_angle(body_state.pose.orientation)
    body_ori_x = X
    body_ori_y = Y
    body_ori_z = Z
    
    body_vel_x = body_state.twist.angular.x
    body_vel_y = body_state.twist.angular.y
    body_vel_z = body_state.twist.angular.z
    
    return body_ori_y,body_vel_y


def get_wheel_state(wheel_name):
    wheel_state = get_state(link_name = wheel_name)
    
    return wheel_state
    
def get_wheel_param():
    global wheel_state
    
    wheel_state = get_wheel_state(wheel_name)
    
    X, Y, Z = utils.quaternion_to_euler_angle(wheel_state.link_state.pose.orientation)
    wheel_pos_x = wheel_state.link_state.pose.position.x
    wheel_ori_x = X
    wheel_ori_y = Y
    wheel_ori_z = Z
    wheel_vel_x = wheel_state.link_state.twist.angular.x
    wheel_vel_y = wheel_state.link_state.twist.angular.y
    wheel_vel_z = wheel_state.link_state.twist.angular.z

    return wheel_pos_x,wheel_ori_x, wheel_ori_y, wheel_ori_z, wheel_vel_x, wheel_vel_y, wheel_vel_z


def print_graph():
    
    
    plt.figure(1)
    plt.subplot(321)
    plt.plot(sec_store, deg_w_store)
    plt.xlabel('sec[s]', fontsize=16)
    # plt.ylabel(r'$\theta_1$[deg]')
    # plt.title('Pitch tilt angle')
    # plt.ylim(-5, 5)
    # plt.xlim(0, 15)
    plt.xticks(fontsize=16)
    plt.yticks(fontsize=16)
    plt.legend([r'$\theta_w$[deg]'],loc='best', fontsize = 20)
    plt.grid(axis='y')
    
    plt.subplot(322)
    plt.plot(sec_store, ddeg_w_store)
    plt.xlabel('sec[s]', fontsize=16)
    # plt.ylabel(r'$\theta_1$[deg]')
    # plt.title('Pitch tilt angle')
    # plt.ylim(-5, 5)
    # plt.xlim(0, 15)
    plt.xticks(fontsize=16)
    plt.yticks(fontsize=16)
    plt.legend([r'$\dot\theta_w$[deg/sec]'],loc='best', fontsize = 20)
    plt.grid(axis='y')
    
    plt.subplot(323)
    plt.plot(sec_store, deg_1_store)
    plt.xlabel('sec[s]', fontsize=16)
    # plt.ylabel(r'$\theta_1$[deg]')
    # plt.title('Pitch tilt angle')
    plt.ylim(-7, 7)
    # plt.xlim(0, 15)
    plt.xticks(fontsize=16)
    plt.yticks(fontsize=16)
    plt.legend([r'$\theta_1$[deg]'],loc='best', fontsize = 20)
    plt.grid(axis='y')
    
    plt.subplot(324)
    plt.plot(sec_store, ddeg_1_store)
    plt.xlabel('sec[s]', fontsize=16)
    # plt.ylabel(r'$\theta_1$[deg]')
    # plt.title('Pitch tilt angle')
    # plt.ylim(-5, 5)
    # plt.xlim(0, 15)
    plt.xticks(fontsize=16)
    plt.yticks(fontsize=16)
    plt.legend([r'$\dot\theta_1$[deg/sec]'],loc='best', fontsize = 20)
    plt.grid(axis='y')
    
    plt.subplot(325)
    plt.plot(sec_store, deg_2_store)
    plt.xlabel('sec[s]', fontsize=16)
    # plt.ylabel(r'$\theta_b$[deg]')
    # plt.title('Pitch tilt angle')
    plt.ylim(-7, 7)
    # plt.xlim(0, 15)
    plt.xticks(fontsize=16)
    plt.yticks(fontsize=16)
    plt.legend([r'$\theta_b$[deg]'],loc='best', fontsize = 20)
    plt.grid(axis='y')
    
    plt.subplot(326)
    plt.plot(sec_store, ddeg_2_store)
    plt.xlabel('sec[s]', fontsize=16)
    # plt.ylabel(r'$\theta_1$[deg]')
    # plt.title('Pitch tilt angle')
    # plt.ylim(-5, 5)
    # plt.xlim(0, 15)
    plt.xticks(fontsize=16)
    plt.yticks(fontsize=16)
    plt.legend([r'$\dot\theta_b$[deg/sec]'],loc='best', fontsize = 20)
    plt.grid(axis='y')
    plt.show()
    
def print_graph1():
    # plt.subplot(222)
    plt.plot(sec_store, deg_1_store)
    plt.xlabel('sec[s]', fontsize=16)
    # plt.ylabel(r'$\theta_b$[deg]')
    # plt.title('Pitch tilt angle')
    plt.ylim(-19, 19)
    plt.xlim(0, 15)
    plt.xticks(fontsize=16)
    plt.yticks(fontsize=16)
    plt.legend([r'$\theta_1$[deg]'],loc='upper right', fontsize = 20)
    plt.grid()
    plt.show()
    
def print_graph2():
    # plt.subplot(222)
    plt.plot(sec_store, deg_2_store)
    plt.xlabel('sec[s]', fontsize=16)
    # plt.ylabel(r'$\theta_b$[deg]')
    # plt.title('Pitch tilt angle')
    plt.ylim(-19, 19)
    plt.xlim(0, 15)
    plt.xticks(fontsize=16)
    plt.yticks(fontsize=16)
    plt.legend([r'$\theta_b$[deg]'],loc='upper right', fontsize = 20)
    plt.grid()
    plt.show()
    
def print_graph_des():
    # plt.figure(1, figsize=(15,12), dpi=150)
    plt.subplot(321);
    plt.plot(sec_store, deg_w_store, sec_store, deg_w_store_des);
    plt.xlabel('sec[s]', fontsize=16)
    plt.xticks(fontsize=16)
    plt.yticks(fontsize=16)
    plt.xlim(0, 15)
    plt.legend([r'$\theta_w$ [deg]',r'$\theta_w^{des}$ [deg]'], loc='best',fontsize = 20)
    plt.grid()

    # plt.figure(1, figsize=(15,12), dpi=150)
    plt.subplot(322);
    plt.plot(sec_store, ddeg_w_store, sec_store, ddeg_w_store_des);
    plt.xlabel('sec[s]', fontsize=16)
    plt.xticks(fontsize=16)
    plt.yticks(fontsize=16)
    plt.xlim(0, 15)
    plt.legend([r'$\dot\theta_w$ [deg/sec]', r'$\dot\theta_w^{ref}$ [deg/sec]'], loc='best',fontsize = 20)
    plt.grid()

    # plt.figure(1, figsize=(15,12), dpi=150)
    plt.subplot(323);
    plt.plot(sec_store, deg_1_store, sec_store, deg_1_store_des);
    plt.xlabel('sec[s]', fontsize=16)
    plt.ylim(-7, 7)
    plt.xticks(fontsize=16)
    plt.yticks(fontsize=16)
    plt.xlim(0, 15)
    plt.legend([r'$\theta_1$ [deg]', r'$\theta_1^{ref}$ [deg]'], loc='best',fontsize = 20)
    plt.grid()

    # plt.figure(1, figsize=(15,12), dpi=150)
    plt.subplot(324);
    plt.plot(sec_store, ddeg_1_store, sec_store, ddeg_1_store_des);
    plt.xlabel('sec[s]', fontsize=16)
    plt.xticks(fontsize=16)
    plt.yticks(fontsize=16)
    plt.ylim(-40, 40)
    plt.xlim(0, 15)
    plt.legend([r'$\dot\theta_1$ [deg/sec]', r'$\dot\theta_1^{ref}$ [deg/sec]'], loc='best',fontsize = 20)
    plt.grid()

    # plt.figure(1, figsize=(15,12), dpi=150)
    plt.subplot(325);
    plt.plot(sec_store, deg_2_store, sec_store, deg_2_store_des);
    plt.xlabel('sec[s]', fontsize=16)
    plt.ylim(-7, 7)
    plt.xticks(fontsize=16)
    plt.yticks(fontsize=16)
    plt.xlim(0, 15)
    plt.legend([r'$\theta_b$ [deg]', r'$\theta_b^{ref}$ [deg]'], loc='best',fontsize = 20)
    plt.grid()

    # plt.figure(1, figsize=(15,12), dpi=150)
    plt.subplot(326);
    plt.plot(sec_store, ddeg_2_store, sec_store, ddeg_2_store_des);
    plt.xlabel('sec[s]', fontsize=16)
    plt.xticks(fontsize=16)
    plt.yticks(fontsize=16)
    plt.ylim(-40, 40)
    plt.xlim(0, 15)
    plt.legend([r'$\dot\theta_b$ [deg/sec]', r'$\dot\theta_b^{ref}$ [deg/sec]'], loc='best',fontsize = 20)
    plt.grid()
    plt.show()
    
def traj():
    tilt_traj = np.load('tilt_traj.npy')
    time_traj = np.load('time.npy')
    traj_pos = np.load('traj_pos.npy')
    traj_vel = np.load('traj_vel.npy')
    time_traj = time_traj.tolist()
    
    traj_des=[]
    for i in range(len(time_traj)-1):
        traj_a = [traj_pos[i], traj_vel[i], tilt_traj[i]]
        traj_des.append(traj_a)

    return traj_des, time_traj
    
# def gimbal_callback(msg):
#     global ankle_pos
#     global ankle_vel
#     global wheel_pos
#     global wheel_vel
    
#     ankle_pos = msg.position[3]
#     ankle_vel = msg.velocity[3]
    
#     wheel_pos = msg.position[4]
#     wheel_vel = msg.velocity[4]
#     print("ankle_pos: ",ankle_pos)
#     print("ankle_vel: ",ankle_vel)
# 1 - Left_flywheel_joint
# 2 - Right_flywheel_joint
# 3 - Right_gimbal_joint
# 4 - ankle_pitch_joint
# 5 - wheel_joint


    
# def Imucallback(msg):
#     X, Y, Z = utils.quaternion_to_euler_angle(msg.orientation)

#     imu_ori_x = X
#     imu_ori_y = Y
#     imu_ori_z = Z
#     imu_vel_x = msg.angular_velocity.x
#     imu_vel_y = msg.angular_velocity.y
#     imu_vel_z = msg.angular_velocity.z
#     print('imu_ori_x: ',imu_ori_x)
    
    
    

    
get_state = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
get_body = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)


body_state = []
body_name = 'wheeled_inverted_pendulum'
body_name_list = [body_name]

wheel_state = []
wheel_name = 'wheel_link'
wheel_name_list = [wheel_name]



sec_store = []
deg_w_store = []
deg_w_store_des = []
ddeg_w_store = []
ddeg_w_store_des = []
deg_1_store = []
deg_1_store_des = []
ddeg_1_store = []
ddeg_1_store_des = []
deg_2_store = []
deg_2_store_des = []
ddeg_2_store = []
ddeg_2_store_des = []

i = 0

# traj_des = traj()
# print(traj_des[0])

# print(len(time_traj))

# A = np.array([[   0, 0 , 0 , 1   ,0  ,  0],
#               [   0, 0, 0, 0, 1, 0 ],
#               [   0, 0, 0, 0, 0, 1 ],
#               [   0, -110.48476107,  148.88129661, 0, 0, 0 ],
#               [   0, 152.43497165, -50.18179263, 0, 0, 0],
#               [   0, -19.32542343, -3.84835859, 0, 0, 0]])

# B = np.array([[   0, 0 ],
#               [   0, 0],
#               [   0, 0],
#               [   0.40632762, -4.0911808],
#               [   -7.22638916, 0.62184574],
#               [   2.107035, 1.0039766]])

# Q = np.array([ [1,0,0,0,0,0],
#                 [0,1,0,0,0,0],
#                 [0,0,1,0,0,0],
#                 [0,0,0,1,0,0],
#                 [0,0,0,0,1,0],
#                 [0,0,0,0,0,1] ])
# R = np.array([ [4,0],
#                 [0,4] ])

# K, S, E = pitch_K_gain(A, B, Q, R)
#######################################################################################################

#######################################################################################################

if __name__ == '__main__':
    try:  
        rospy.init_node('Double_Inverted_Pendulum', anonymous=True)
        pub_w = rospy.Publisher('/wheeled_inverted_pendulum/wheel/command', Float64, queue_size=100)
        pub_2 = rospy.Publisher('/wheeled_inverted_pendulum/ankle_pitch/command', Float64, queue_size=100)
        rate = rospy.Rate(10000000)
        
        gazebo_setting()
        RAD2DEG = 180/np.pi
        traj_des, time_traj = traj()
        # traj_des = [time_traj[i], traj_pos[i], traj_vel[i], tilt_traj[i]]
        
        link1_ori_y,link2_ori_y, link1_vel_y, link2_vel_y = get_cur_deg_vel()
        wheel_pos_x,wheel_ori_x, wheel_ori_y, wheel_ori_z, wheel_vel_x, wheel_vel_y, wheel_vel_z = get_wheel_param()

        # x0 = np.array([wheel_ori_y, link1_ori_y, link2_ori_y-0.745238866846359,wheel_vel_y,link1_vel_y,link2_vel_y])
        x0 = np.array([link1_ori_y, link2_ori_y-0.745238866846359,wheel_vel_y,link1_vel_y,link2_vel_y])

        mpc_model, estimator, u0 = mpc.set_model(x0)
        
        cur_time = time.time()    
        sec_time = time.time() 
        
        # rospy.Subscriber("/wheeled_inverted_pendulum/joint_states", JointState, gimbal_callback)         
        while True:
            
            last_time = cur_time
            cur_time = time.time()
            sec_cur_time = time.time()
            dt = cur_time - last_time             
            sec =  sec_cur_time - sec_time
            
            u = mpc_model.make_step(x0)
            
            print('u:, ',u)
            print(i)
            
            u1 = u[0]
            u2 = u[1]
            
            pub_w.publish(u1)
            pub_2.publish(u2)
            
            deg_w_store.append((wheel_pos_x)/0.069)
            ddeg_w_store.append((wheel_vel_y)*RAD2DEG)
            deg_1_store.append((link1_ori_y)*RAD2DEG)
            ddeg_1_store.append((link1_vel_y)*RAD2DEG)
            deg_2_store.append((link2_ori_y-0.745238866846359)*RAD2DEG)
            ddeg_2_store.append((link2_vel_y)*RAD2DEG)
            # print(len(deg_w_store))
            
            # if i<len(time_traj)-1:
                
            #     deg_w_store_des.append((traj_des[i][0]*1500)/0.069)
            #     ddeg_w_store_des.append((traj_des[i][1]*20000)*RAD2DEG)
            #     deg_1_store_des.append(0)
            #     ddeg_1_store_des.append(0)
            #     deg_2_store_des.append((traj_des[i][2]*0.7)*RAD2DEG)
            #     ddeg_2_store_des.append(0)
            # else:
            #     deg_w_store_des.append((traj_des[-1][0]*1500)/0.069)
            #     ddeg_w_store_des.append(0)
            #     deg_1_store_des.append(0)
            #     ddeg_1_store_des.append(0)
            #     deg_2_store_des.append(0)
            #     ddeg_2_store_des.append(0)
                
            sec_store.append(sec)
            # 900
            # 1500
            # if i == 1500:
            #     print_graph1()
            #     print_graph2()
            #     print_graph_des()
                # sys.stdout = open('deg_1_store.txt','w')
                # print(deg_1_store)
                # sys.stdout = open('deg_2_store.txt','w')
                # print(deg_2_store)
                
                
            rate.sleep()  
            
            link1_ori_y,link2_ori_y, link1_vel_y, link2_vel_y = get_cur_deg_vel()
            wheel_pos_x,wheel_ori_x, wheel_ori_y, wheel_ori_z, wheel_vel_x, wheel_vel_y, wheel_vel_z = get_wheel_param()

            # if i < 1:
            #     wheel_ori_x, wheel_ori_y, wheel_ori_z, wheel_vel_x, wheel_vel_y, wheel_vel_z = get_wheel_param()

            #     y_step = np.array([wheel_ori_y, link1_ori_y, 0,wheel_vel_y,link1_vel_y,link2_vel_y])
            # else:
                
            #     # print(link2_ori_y-0.745238866846359)
            #     y_step = np.array([wheel_pos, link1_ori_y, ankle_pos,wheel_vel,link1_vel_y,link2_vel_y])
            #     # deg_1_store.append((link1_ori_y)*RAD2DEG)
            #     # deg_2_store.append((ankle_pos)*RAD2DEG)
            #     # sec_store.append(sec)
            # y_step = np.array([wheel_ori_y, link1_ori_y, link2_ori_y-0.745238866846359,wheel_vel_y,link1_vel_y,link2_vel_y])
            y_step = np.array([link1_ori_y, link2_ori_y-0.745238866846359,wheel_vel_y,link1_vel_y,link2_vel_y])

            print('-------------------------------------')
            print('state: ', y_step)
            print('dt: ',dt)
            print('-------------------------------------')
            
            # if i % 10 == 0:
            #     print('-------------------------------------')
            #     print('state: ', y_step)
            #     print('dt: ',dt)
            #     print('-------------------------------------')
            
            x0 = estimator.make_step(y_step)
            
            i= i + 1
            # print('u1: ', u1)
            

    except rospy.ROSInterruptException:
        pass
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


def gazebo_setting():
    os.system('rosservice call /gazebo/set_physics_properties "{time_step: 0.001, max_update_rate: 1000.0, gravity: {x: 0.0, y: 0.0, z: -9.8}, ode_config: {auto_disable_bodies: False, sor_pgs_precon_iters: 0, sor_pgs_iters: 200, sor_pgs_w: 1.0, sor_pgs_rms_error_tol: 0.0, contact_surface_layer: 0.001, contact_max_correcting_vel: 100.0, cfm: 0.0, erp: 0.2, max_contacts: 20}}"')
    # os.system('rosservice call gazebo/unpause_physics')
    rospy.loginfo("Simulation Start")
    
def roll_K_gain_R():
        
    K, S, E = control.lqr(A, B, Q, R)
    
    return K, S, E

def rot_x(theta):
    rot_x = np.array([[1,0,0],
                      [0,np.cos(theta),-np.sin(theta)],
                      [0,np.sin(theta),np.cos(theta)]])
    return rot_x

def rot_y(theta):
    rot_y = np.array([[np.cos(theta),0,np.sin(theta)],
                       [0,1,0],
                       [-np.sin(theta),0,np.cos(theta)]])
    return rot_y

def rot_z(theta):
    rot_z = np.array([[np.cos(theta),-np.sin(theta),0],
                       [np.sin(theta),np.cos(theta),0],
                       [0,0,1]])
    return rot_z

def quaternion_to_euler_angle(msg):
    x = msg.x
    y = msg.y
    z = msg.z
    w = msg.w
    
    ysqr = y * y
    
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = math.atan2(t3, t4)
    
    return X, Y, Z
        
    
def get_model_state(body_name):
    
    body_state = get_body(model_name = body_name)
    
    return body_state

def get_body_ori():
    global body_state
    
    body_state = get_model_state(body_name)
    
    X, Y, Z = quaternion_to_euler_angle(body_state.pose.orientation)
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
    
def get_link_state(link_name_main, reference_frame):
    
    get_state = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
    link_state = get_state(link_name= link_name_main, reference_frame = reference_frame)
    
    return link_state

def get_link_param(link_name_main, reference_frame):
    
    global link_state
    
    link_state = get_link_state(link_name_main, reference_frame)
    
    X, Y, Z = quaternion_to_euler_angle(link_state.link_state.pose.orientation)
    link_ori_x = X
    link_ori_y = Y
    link_ori_z = Z
    
    link_vel_x = link_state.link_state.twist.angular.x
    link_vel_y = link_state.link_state.twist.angular.y
    link_vel_z = link_state.link_state.twist.angular.z
     
    return link_ori_x, link_vel_x


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
    
    return body_ori_x,body_vel_x

def theta_target(gimbal_deg):
    
    DEG2RAD = np.pi/180
    gimbal_deg = gimbal_deg*DEG2RAD
    
    theta_target = np.array([gimbal_deg])
    
    return theta_target

# def callback(data):
#     global z_com
    
#     CoMlist = data.data
#     z_com = CoMlist[0]

def print_graph():

    plt.plot(sec_store, deg_store)

    plt.xlabel('sec[s]', fontsize=16)
    # plt.ylabel('tilt angle[deg]')
    # plt.title('Roll tilt angle')
    plt.ylim(-19, 19)
    plt.xlim(0, 15)
    plt.xticks(fontsize=16)
    plt.yticks(fontsize=16)
    plt.legend([r'$\theta_r$[deg]'],loc='upper right', fontsize = 20)
    plt.grid(axis='y')

    plt.show()
    
def gimbal_callback(msg):
    global gimbal_pos
    global gimbal_vel
    # rate = rospy.Rate(100)
    gimbal_pos = msg.position[2]
    gimbal_vel = msg.velocity[2]
    print('gimbal_pos: ', gimbal_pos)
    print('----------------------')
    # rate.sleep()
    
# def global_gimbal():
#     global gimbal_pos
#     global gimbal_vel
#     return gimbal_pos, gimbal_vel

#################################################################################################    
z_com = 0.5004170132265042
A, B, C, D = ser.Cal_Roll_SS(z_com)
# A = np.array([[0,0,1,0],
#             [0,0,0,1],
#             [82.04066448,0,0,-1.91506379],
#             [0,0,497.19879333, 0]])

# B = np.array([[0],[0],[0],[84.03361345]])

# C = np.eye(4)

# D = np.array([[0], [0], [0], [0]])
# q = [theta_R, theta_gb, theta_Rd, theta_gbd]
Q = sp.Matrix([ [1,    0,    0,    0],
                [0,    10,    0,    0],
                [0,    0,    1,    0],
                [0,    0,    0,    1]])

R = sp.Matrix([ [1] ])

K, S, E = roll_K_gain_R()
ss0 = [A, B, C, D]
sys0 = control.ss(*[pl.array(mat_i).astype(float) for mat_i in ss0])
sysc = sys0.feedback(K) 


RAD2DEG = 180/np.pi       
    
loop_cnt = 1

get_body = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

gimbal_state = []
Left_gimbal_name = 'Left_gimbal'
Right_gimbal_name = 'Right_gimbal'
Left_flywheel_name = 'Left_flywheel'
Right_flywheel_name = 'Right_flywheel'
gimbal_name_list = [Left_gimbal_name, Right_gimbal_name, Left_flywheel_name, Right_flywheel_name]

mid_name = 'knee_ankle_link'
low_name = 'ankle_roll_yaw_link'
top_name = 'cmg'
link_name_list = [mid_name, low_name, top_name]

 
body_state = []
body_name = 'wheeled_inverted_pendulum'
body_name_list = [body_name]

deg_store = []
sec_store = []

print('K: ', K)
#################################################################################################
    
if __name__ == '__main__':
    try:
        rospy.wait_for_service('gazebo/get_model_state')
        rospy.init_node('Roll_Controller', anonymous=False) 

        pub_Rgb = rospy.Publisher('/wheeled_inverted_pendulum/right_gimbal/command', Float64, queue_size=100)
        pub_Lfw = rospy.Publisher('/wheeled_inverted_pendulum/left_flywheel/command', Float64, queue_size=100)
        pub_Rfw = rospy.Publisher('/wheeled_inverted_pendulum/right_flywheel/command', Float64, queue_size=100)
        # rospy.Subscriber('CoM_Roll', Float64MultiArray, callback)
        

       
        gazebo_setting()

        gimbal_ori = 0
        gimbal_vel = 0
        rate = rospy.Rate(100)
        rate1 = rospy.Rate(1)
        rpm = 5000
        flywheel_ang_vel = (rpm * 2 * np.pi)/60        
        flywheel_vel = np.array([flywheel_ang_vel])
        print(flywheel_vel)
        
        
        ref = 0
        t = 0.01
        # rate1.sleep()

        cur_time = time.time()  
        sec_time = time.time()  
        rospy.Subscriber("/wheeled_inverted_pendulum/joint_states", JointState, gimbal_callback)         
        while not rospy.is_shutdown():    
            # global gimbal_pos
            # global gimbal_vel
            pub_Lfw.publish(flywheel_vel[0])
            pub_Rfw.publish(-flywheel_vel[0])
        
            last_time = cur_time
            cur_time = time.time()
            sec_cur_time = time.time()
            dt = cur_time - last_time 
            sec =  sec_cur_time - sec_time
            
            link_ori_x, link_vel_x = get_link_param(gimbal_name_list[1], 'world')
                        
            # link_ori_x, link_ori_y, link_ori_z = get_link_ori(link_name_list[2], 'world')
            # link_vel_x, link_vel_y, link_vel_z = get_link_vel(link_name_list[2], 'world')
            body_ori_x,body_vel_x = get_body_param()
            
            
            if loop_cnt < 2:
                x0 = np.array([body_ori_x, link_ori_x, body_vel_x, link_vel_x])
                print('Gimbal_angle_r      (deg): ', link_ori_x*RAD2DEG)
            else:
                x0 = np.array([body_ori_x, gimbal_pos, body_vel_x, link_vel_x])
                print('Gimbal_angle_r      (deg): ', gimbal_pos*RAD2DEG)
            
            xd = np.array([0, 0, 0, 0])
            # gimbal_pos
            # print(x0)
            u_R = - K @ (x0 - xd) 
            
            pub_Rgb.publish(u_R)
                
            # deg_store.append(body_ori_x*RAD2DEG)
            # sec_store.append(sec)
            
            # if loop_cnt % 10 == 0:
                # flywheel_vel_x, flywheel_vel_y, flywheel_vel_z= get_link_param(gimbal_name_list[2], gimbal_name_list[0])
                
            
            # print('Gimbal_angle_l      (deg): ', gimbal_ori_left_z*RAD2DEG)
            # print('FLywheel_velocity (rad/s): ', abs(flywheel_vel_y))
            print('Roll                (deg): ', body_ori_x)
            print('dt: ', dt )
            # print('Yaw                 (deg): ', link_ori_z*RAD2DEG)
            print('====================================')

            loop_cnt= loop_cnt + 1
            # print(deg_store)
            rate.sleep()
            
            # if loop_cnt == 1500:
            #     print_graph()
            #     sys.stdout = open('deg_store.txt','w')
            #     print(deg_store)
                
            
    except rospy.ROSInterruptException:
        pass

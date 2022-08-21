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
import init_traj
import Cal_joint as cj


def gazebo_setting():
    os.system('rosservice call /gazebo/set_physics_properties "{time_step: 0.001, max_update_rate: 1000.0, gravity: {x: 0.0, y: 0.0, z: -9.8}, ode_config: {auto_disable_bodies: False, sor_pgs_precon_iters: 0, sor_pgs_iters: 200, sor_pgs_w: 1.0, sor_pgs_rms_error_tol: 0.0, contact_surface_layer: 0.001, contact_max_correcting_vel: 100.0, cfm: 0.0, erp: 0.2, max_contacts: 20}}"')
    # os.system('rosservice call gazebo/unpause_physics')
    rospy.loginfo("Simulation Start")

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

    ankle_link = 'ankle_link'
    knee_ankle_link = 'knee_ankle_link'
    hip_to_knee_link = 'hip_to_knee_link'
    cmg_link = 'cmg'
    link_name_list = [ankle_link, knee_ankle_link, hip_to_knee_link, cmg_link]

    link1_ori_x, link1_ori_y, link1_ori_z, link1_vel_x, link1_vel_y, link1_vel_z = get_link_ori_vel(link_name_list[0], 'world')
    link2_ori_x, link2_ori_y, link2_ori_z, link2_vel_x, link2_vel_y, link2_vel_z = get_link_ori_vel(link_name_list[1], link_name_list[0])
    link3_ori_x, link3_ori_y, link3_ori_z, link3_vel_x, link3_vel_y, link3_vel_z = get_link_ori_vel(link_name_list[2], link_name_list[1])
    link_4ori_x, link4_ori_y, link4_ori_z, link4_vel_x, link4_vel_y, link4_vel_z = get_link_ori_vel(link_name_list[3], link_name_list[2])

    link3_ori_y = -(link2_ori_y + link4_ori_y)

    thetalist = np.array([[-link1_ori_y], [-link2_ori_y], [-link3_ori_y], [-link4_ori_y]])
    dthetalist = np.array([[-link1_vel_y], [-link2_vel_y], [-link3_vel_y], [-link4_vel_y]])
 
    return thetalist, dthetalist

def get_wheel_state(wheel_name):
    wheel_state = get_state(link_name = wheel_name)
    
    return wheel_state
    
def get_wheel_param():
    global wheel_state
    
    wheel_state = get_wheel_state(wheel_name)
    
    X, Y, Z = utils.quaternion_to_euler_angle(wheel_state.link_state.pose.orientation)
    wheel_ori_x = X
    wheel_ori_y = Y
    wheel_ori_z = Z
    wheel_vel_x = wheel_state.link_state.twist.angular.x
    wheel_vel_y = wheel_state.link_state.twist.angular.y
    wheel_vel_z = wheel_state.link_state.twist.angular.z

    return wheel_ori_x, wheel_ori_y, wheel_ori_z, wheel_vel_x, wheel_vel_y, wheel_vel_z

def horizon_traj(h_cur, horizon_cur, horizon_end, thetalistd):
    time, traj = utils.Trapezoidal_Traj_Gen_Given_Amax_and_T(1,2,0.03)
    horizon_path = utils.Path_Gen(horizon_cur, horizon_end, traj[:,0])
    m1 = 2.486 + 0.3
    m2 = 1.416
    m3 = 1.739
    m4 = 16.09

    L1 = 0.171
    L2 = 0.28
    L3 = 0.280
    L4 = 0.346

    L1c = L1/2
    L2c = L2 - 0.045289
    L3c = L3 - 0.18878

    h = h_cur
    q1list = np.array([0])
    q2list = np.array([thetalistd[1]])
    q3list = np.array([thetalistd[2]])
    q4list = np.array([thetalistd[3]])

    for i in range(0, len(time)-1):
        

        A = (h-L4-L1)/L2
        B = horizon_path[i]/L2

        theta_2, theta_3 = sp.symbols('theta_2, theta_3')
        f1 = sp.Eq(sp.sin(theta_2)+sp.sin(theta_3),float(A))
        f2 = sp.Eq(sp.cos(theta_2)+sp.cos(theta_3),float(B))
        # f3 = sp.Eq(sp.sin(theta_2)+sp.sin(theta_3),float(E))

        sol = sp.solve([f1,f2])
        solu = sol[0]

        theta_2 = solu[theta_2]
        theta_3 = solu[theta_3]
        q1 = np.pi/2
        q2 = theta_2-q1
        q3 = theta_3-q1-q2
        q4 = -(q2+q3)

        q1list = np.vstack((q1list,0))
        q2list = np.vstack((q2list,q2))
        q3list = np.vstack((q3list,q3))
        q4list = np.vstack((q4list,q4))

    return q1list, q2list, q3list, q4list

def CTC(thetalist_des,thetalist,thetalist_prev, dthetalistd, dt):
    m1 = 2.486 + 0.3
    m2 = 1.416
    m3 = 1.739
    m4 = 16.09

    L1 = 0.171
    L2 = 0.28
    L3 = 0.280
    L4 = 0.346

    L1c = L1/2
    L2c = L2 - 0.045289
    L3c = L3 - 0.18878
    L4c = 0.158527
    
    theta_1 = thetalist[0]
    theta_2 = thetalist[1]
    theta_3 = thetalist[2]
    theta_4 = thetalist[3]
    
    theta_1_prev = thetalist_prev[0]
    theta_2_prev = thetalist_prev[1]
    theta_3_prev = thetalist_prev[2]
    theta_4_prev = thetalist_prev[3]
    
    dtheta_1 = dthetalistd[0]
    dtheta_2 = dthetalistd[1]
    dtheta_3 = dthetalistd[2]
    dtheta_4 = dthetalistd[3]
    
    eint = np.array([[0.2], [0.2], [0.2], [0.2]])
    g = np.array([0, 0, -9.8])
    
    M01 = np.array([[1, 0, 0,       0],
                    [0, 1, 0,       0],
                    [0, 0, 1,       0],
                    [0, 0, 0,       1]])
    
    M12 = np.array([[1, 0, 0,        0],
                    [0, 1, 0,       L1c],
                    [0, 0, 1,        0],
                    [0, 0, 0,        1]])
    
    M23 = np.array([[ 1, 0, 0,      0],
                    [ 0, 1, 0,      L2c],
                    [ 0, 0, 1,       0],
                    [ 0, 0, 0,       1]])
    
    M34 = np.array([[1, 0, 0,       0],
                    [0, 1, 0,       L3c],
                    [0, 0, 1,      0],
                    [0, 0, 0,       1]])
    
    M45 = np.array([[1, 0, 0,       0],
                    [0, 1, 0,      L4c],
                    [0, 0, 1,       0],
                    [0, 0, 0,       1]])
    
    G1 = np.diag([0.00613516, 0.00614941, 0.004981955, m1, m1, m1]) 
    G2 = np.diag([0.010192583, 0.009599672, 0.002390603, m2, m2, m2])
    G3 = np.diag([0.008595913, 0.007917693, 0.002944951, m3, m3, m3])
    G4 = np.diag([0.306643651, 0.257336003, 0.140868631, m4, m4, m4]) 
    
    Glist = np.array([G1, G2, G3, G4])
    Mlist = np.array([M01, M12, M23, M34, M45])
    Slist = np.array([[0, 0, 1,      0, 0,     0],
                      [0, 0, -1,   -L1, 0,     0],
                      [0, 0, 1, L1 + L2, 0, 0.0],
                      [0, 0, -1, -(L1 + L2 +L3 ), 0 , 0.0]]).T
    Kp = 0.1
    Ki = 0.1
    Kd = 0.1
    
    thetalistd = np.array([[0], [thetalist_des[1]], [thetalist_des[2]], [thetalist_des[3]]],dtype=float)
    # thetalistd = np.array([traj_th1[i+1], traj_th2[i+1], traj_th3[i+1], traj_th4[i+1]],dtype=float)
    dthetalist = np.array([(theta_1-theta_1_prev)/dt, (theta_2-theta_2_prev)/dt, (theta_3-theta_3_prev)/dt, (theta_4-theta_4_prev)/dt],dtype=float)
    ddthetalistd = np.array([(dtheta_1-dthetalist[0])/dt, (dtheta_2-dthetalist[1])/dt, (dtheta_3-dthetalist[2])/dt, (dtheta_4-dthetalist[3])/dt],dtype=float)
    
    e = np.subtract(thetalistd, thetalist)
    MassMatrix = mr.MassMatrix(thetalist, Mlist, Glist, Slist)
    error = Kp * e + Ki * (np.array(eint) + e) \
            + Kd * (np.subtract(dthetalistd, dthetalist))
            
    # print(error)
    # print('------------------')  
    # print(np.array(eint) + e) 
    # print(np.subtract(dthetalistd, dthetalist)) 
    # print('------------------')       

    
    inv_dyn = np.array([mr.InverseDynamics(thetalist, dthetalist, ddthetalistd, g, \
                        [0, 0, 0, 0, 0, 0], Mlist, Glist, Slist)]).T
    # print(inv_dyn)
    torque = np.dot(MassMatrix,error)+inv_dyn  
    return torque
    


def CTC_traj(q1list, q2list, q3list, q4list):
    time_, traj_ = utils.Trapezoidal_Traj_Gen_Given_Amax_and_T(1,2,0.03)
    # thetalist = get_cur_deg()
    
    m1 = 2.486 + 0.3
    m2 = 1.416
    m3 = 1.739
    m4 = 16.09

    L1 = 0.171
    L2 = 0.28
    L3 = 0.280
    L4 = 0.346

    L1c = L1/2
    L2c = L2 - 0.045289
    L3c = L3 - 0.18878
    L4c = 0.158527

    traj_th1 = q1list
    traj_th2 = q2list
    traj_th3 = q3list
    traj_th4 = q4list

    eint = np.array([[0.2], [0.2], [0.2], [0.2]])
    g = np.array([0, 0, -9.8])
    
    M01 = np.array([[1, 0, 0,       0],
                    [0, 1, 0,       0],
                    [0, 0, 1,       0],
                    [0, 0, 0,       1]])
    
    M12 = np.array([[1, 0, 0,        0],
                    [0, 1, 0,       L1c],
                    [0, 0, 1,        0],
                    [0, 0, 0,        1]])
    
    M23 = np.array([[ 1, 0, 0,      0],
                    [ 0, 1, 0,      L2c],
                    [ 0, 0, 1,       0],
                    [ 0, 0, 0,       1]])
    
    M34 = np.array([[1, 0, 0,       0],
                    [0, 1, 0,       L3c],
                    [0, 0, 1,      0],
                    [0, 0, 0,       1]])
    
    M45 = np.array([[1, 0, 0,       0],
                    [0, 1, 0,      L4c],
                    [0, 0, 1,       0],
                    [0, 0, 0,       1]])
    
    G1 = np.diag([0.00613516, 0.00614941, 0.004981955, m1, m1, m1]) 
    G2 = np.diag([0.010192583, 0.009599672, 0.002390603, m2, m2, m2])
    G3 = np.diag([0.008595913, 0.007917693, 0.002944951, m3, m3, m3])
    G4 = np.diag([0.306643651, 0.257336003, 0.140868631, m4, m4, m4]) 
    
    Glist = np.array([G1, G2, G3, G4])
    Mlist = np.array([M01, M12, M23, M34, M45])
    Slist = np.array([[0, 0, 1,      0, 0,     0],
                      [0, 0, -1,   -L1, 0,     0],
                      [0, 0, 1, L1 + L2, 0, 0.0],
                      [0, 0, -1, -(L1 + L2 +L3 ), 0 , 0.0]]).T
    Kp = 10
    Ki = 0.1
    Kd = 0.1
    dt = 0.015

    # thetalist_prev, dthetalist_prev = get_cur_deg_vel()
    rate = rospy.Rate(1000)
    cur_time = time.time()    
    for i in range(0, len(time_)-1):
        prev_time = cur_time
        cur_time = time.time()
        ddt = cur_time - prev_time 
        print(ddt)
        
        thetalist, dthetalist = get_cur_deg_vel()
        # print(thetalist)
        thetalistd = np.array([traj_th1[i], traj_th2[i], traj_th3[i], traj_th4[i]],dtype=float)
        # thetalistd = np.array([traj_th1[i+1], traj_th2[i+1], traj_th3[i+1], traj_th4[i+1]],dtype=float)
        dthetalistd = np.array([(traj_th1[i+1]-traj_th1[i])/dt, (traj_th2[i+1]-traj_th2[i])/dt, (traj_th3[i+1]-traj_th3[i])/dt, (traj_th4[i+1]-traj_th4[i])/dt],dtype=float)
        ddthetalistd = np.array([(dthetalistd[0]-dthetalist[0])/dt, (dthetalistd[1]-dthetalist[1])/dt, (dthetalistd[2]-dthetalist[2])/dt, (dthetalistd[3]-dthetalist[3])/dt],dtype=float)

        e = np.subtract(thetalistd, thetalist)
        MassMatrix = mr.MassMatrix(thetalist, Mlist, Glist, Slist)
        error = Kp * e + Ki * (np.array(eint) + e) \
                + Kd * (np.subtract(dthetalistd, dthetalist))
        # print('------------------')  
        # print(np.array(eint) + e) 
        # print(np.subtract(dthetalistd, dthetalist)) 
        # print('------------------')       

        
        inv_dyn = np.array([mr.InverseDynamics(thetalist, dthetalist, ddthetalistd, g, \
                            [0, 0, 0, 0, 0, 0], Mlist, Glist, Slist)]).T
        torque = np.dot(MassMatrix,error)+inv_dyn       
        print(torque)
        # torquelist = torque.astype(np.float64)
        # print(type(torquelist))
        # print(type(torquelist[1]))
        pub_2.publish(torque[1])
        pub_3.publish(torque[2])
        pub_4.publish(torque[3])
        rate.sleep()

def joint_torque():
    q1, q2, q3, q4, torque = init_traj.joint_traj()
    
    rate = rospy.Rate(100)
    
    for i in range(0,len(torque)-1):
        torque2 = torque[i][1]
        torque3 = torque[i][2]
        torque4 = torque[i][3]
        pub_2.publish(torque2)
        pub_3.publish(torque3)
        pub_4.publish(torque4)
        
        rate.sleep()
    
get_state = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)

wheel_state = []
wheel_name = 'wheel_link'
wheel_name_list = [wheel_name]

deg_store = []
sec_store = []
loop_cnt = 0

#######################################################################################################

#######################################################################################################

if __name__ == '__main__':
    try:  
        rospy.init_node('CTC_Calculator', anonymous=True)
        pub_w = rospy.Publisher('/wheeled_inverted_pendulum/wheel/command', Float64, queue_size=100)
        pub_2 = rospy.Publisher('/wheeled_inverted_pendulum/ankle_pitch/command', Float64, queue_size=100)
        pub_3 = rospy.Publisher('/wheeled_inverted_pendulum/knee/command', Float64, queue_size=100)
        pub_4 = rospy.Publisher('/wheeled_inverted_pendulum/hip_pitch/command', Float64, queue_size=100)
        rate = rospy.Rate(100)
        gazebo_setting()
        RAD2DEG = 180/np.pi

        # ddt = 0.017

        # q1d = 0
        # q2d = 0.745239
        # q3d = -1.6307
        # q4d = 0.885463
        
        thetalist_des, h = cj.get_init_pos(0.9)
        x,z, x_com, z_com, I_by, theta_P = cj.get_end_point(thetalist_des[0], thetalist_des[1], thetalist_des[2])
        # s0, z, theta_P = get_end_point(thetalistd[0], thetalistd[1], thetalistd[2], thetalistd[3])
        # print('Start Calculating Horizon Trajectory!!')
        # q1list, q2list,q3list, q4list= horizon_traj(h, s0, 0.03, thetalistd)
        # q1list, q2list,q3list, q4list = init_traj.joint_traj()
        # print('Calculate Horizon Trajectory!!')
        # CTC_traj(q1list, q2list, q3list, q4list)
        # joint_torque()
        
        # thetalist_prev, dthetalist_prev = get_cur_deg_vel()
        cur_time = time.time()    
        sec_time = time.time() 
        # dt = 0.015

        while True:
            last_time = cur_time
            cur_time = time.time()
            # sec_cur_time = time.time()
            dt = cur_time - last_time 
            print('dt: ',dt)
            # sec =  sec_cur_time - sec_time
            thetalist, dthetalist = get_cur_deg_vel()
            wheel_ori_x, wheel_ori_y, wheel_ori_z, wheel_vel_x, wheel_vel_y, wheel_vel_z = get_wheel_param()
            
            x,z, x_com, z_com, I_by, theta_P = cj.get_end_point(thetalist[1], thetalist[2], thetalist[3])
            # torque = CTC(thetalist_des,thetalist,thetalist_prev,dthetalist_prev, dt)
            
            # thetalist_prev = thetalist
            # print('torque: ',torque)
            x0 = np.array([wheel_ori_y,thetalist[0], theta_P,wheel_vel_y,dthetalist[0],])
            u = -K @ ( x0 )

            pub_2.publish(thetalist_des[0])
            pub_3.publish(thetalist_des[1])
            pub_4.publish(thetalist_des[2])
            pub_w.publish(u)

            if loop_cnt % 10 == 0:
                print('thetalist: ', thetalist)
                print('dthetalist: ', dthetalist)
                print('Wheel_velocity  (rad/s): ', wheel_vel_y)
                # print('Pitch             (deg): ', theta_P*RAD2DEG)

                print('====================================')  
            
            
            loop_cnt= loop_cnt + 1

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
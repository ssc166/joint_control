import sympy as sp
import pylab as pl
import control
import math
import numpy as np
import WIP_utils as utils


def get_init_pos(h):
    

    m2 = 1.416
    m3 = 1.739
    m4 = 16.09

    L1 = 0.171
    L2 = 0.28
    L3 = 0.28
    L4 = 0.346

    L2c = L2 - 0.045289
    L3c = L3 - 0.18878
    
    y = h-L1

    theta_3, theta_2 = sp.symbols('theta_3, theta_2')

    A = (y-L4)/L2
    B = m2*L2c + m3*L2+m4*L2
    C = m3*L3c + m4*L3

    f1 = sp.Eq(sp.sin(theta_2)+sp.sin(theta_3),float(A))
    f2 = sp.Eq(float(B)*sp.cos(theta_2)+float(C)*sp.cos(theta_3),0)
    sol = sp.solve([f1,f2])
    # sol
    solu = sol[0]

    theta_2 = solu[theta_2]
    theta_3 = solu[theta_3]
    # theta_2, theta_3
    q2 = theta_2-np.pi/2
    q3 = theta_3-theta_2
    q4 = np.pi/2- theta_3

    thetalistd = np.array([q2, q3, q4])
    
    return thetalistd

def get_end_point(q2, q3, q4):
    m1 = 2.486 + 0.3
    m2 = 1.416
    m3 = 1.739
    m4 = 16.09

    L1 = 0.171
    L2 = 0.28
    L3 = 0.28
    L4 = 0.346

    L1c = L1
    L2c = L2 - 0.045289
    L3c = L3 - 0.18878
    L4c = 0.17188

    I_2 =0.009599672
    I_3 =0.007917693
    I_4 =0.257336003

    theta_2 = np.pi/2 + q2
    theta_3 = np.pi/2 + q2 + q3
    theta_4 = np.pi/2 + q2 + q3 + q4

    x = L2 * np.cos(float(theta_2)) + L3 * np.cos(float(theta_3)) + L4 * np.cos(float(theta_4))
    z = L2 * np.sin(float(theta_2)) + L3 * np.sin(float(theta_3)) + L4 * np.sin(float(theta_4))

    x_2 = L2c * np.cos(float(theta_2))
    x_3 = L2 * np.cos(float(theta_2)) + L3c*np.cos(float(theta_3))
    x_4 = L2 * np.cos(float(theta_2)) + L3 * np.cos(float(theta_3)) + L4c * np.cos(float(theta_4))
    x_com = (m2*x_2 + m3*x_3+ m4*x_4) / (m2 + m3 + m4)

    z_2c = L2c * np.sin(float(theta_2))
    z_3c = L2 * np.sin(float(theta_2)) + L3c*np.sin(float(theta_3))
    z_4c = L2 * np.sin(float(theta_2)) + L3 * np.sin(float(theta_3)) + L4c * np.sin(float(theta_4))

    z_com = (m2*z_2c + m3*z_3c + m4*z_4c) / (m2 + m3 + m4)
    
    l = np.sqrt(float(x_com)**2 + float(z_com)**2)
    theta_P = np.arctan(float(x_com)/float(z_com))

    I_by =  I_2 + I_3 + I_4 +  m2*(z_com-z_2c)**2 +   m3*(z_com-z_3c)**2 +  m4*(z_com-z_4c)**2 
    return x,z, x_com, z_com, I_by, l,theta_P

def horizon_angle_traj(a, t, time_step,thetalistd, horizon_path, z_com):
    time, traj = utils.Trapezoidal_Traj_Gen_Given_Amax_and_T(a,t,time_step)
    
    m1 = 2.486 + 0.3
    m2 = 1.416
    m3 = 1.739
    m4 = 16.09
    m_b = m2+m3+m4

    L1 = 0.171
    L2 = 0.28
    L3 = 0.280
    L4 = 0.346

    L2c = L2 - 0.045289
    L3c = L3 - 0.18878
    L4c = 0.17188
    
    q2list = np.array([thetalistd[0]],dtype=float)
    q3list = np.array([thetalistd[1]],dtype=float)
    q4list = np.array([thetalistd[2]],dtype=float)
    
    for i in range(0, len(time)-1):

        A = m2*L2c + m3*L2+m4*L2
        B = m3*L3c + m4*L3
        C = horizon_path[i] * m_b
        D = m2*L2c + m3*L2+m4*L2
        E = m3*L3c + m4*L3
        F = z_com * m_b - m4*L4c

        theta_2, theta_3 = sp.symbols('theta_2, theta_3')
        f1 = sp.Eq(float(A)*sp.cos(theta_2)+float(B)*sp.cos(theta_3),float(C))
        f2 = sp.Eq(float(D)*sp.sin(theta_2)+float(E)*sp.sin(theta_3),float(F))
        # f3 = sp.Eq(sp.sin(theta_2)+sp.sin(theta_3),float(E))

        sol = sp.solve([f1,f2])
        sol
        solu = sol[0]

        theta_2 = solu[theta_2]
        theta_3 = solu[theta_3]
        # q1 = np.pi/2
        q2 = theta_2-np.pi/2
        q3 = theta_3-theta_2
        q4 = np.pi/2- theta_3
        
        q2list = np.vstack((q2list,q2),dtype=float)
        q3list = np.vstack((q3list,q3),dtype=float)
        q4list = np.vstack((q4list,q4),dtype=float)
        
    return q2list, q3list, q4list

def end_traj(q2list, q3list, q4list):
    m1 = 2.486 + 0.3
    m2 = 1.416
    m3 = 1.739
    m4 = 16.09

    L1 = 0.171
    L2 = 0.28
    L3 = 0.28
    L4 = 0.346

    L1c = L1
    L2c = L2 - 0.045289
    L3c = L3 - 0.18878
    L4c = 0.17188

    I_2 =0.009599672
    I_3 =0.007917693
    I_4 =0.257336003 
    
    theta_20 = np.pi/2 + q2list[0]
    theta_30 = np.pi/2 + q2list[0] + q3list[0]
    theta_40 = np.pi/2 + q2list[0] + q3list[0] + q4list[0]

    x = L2 * np.cos(float(theta_20)) + L3 * np.cos(float(theta_30)) + L4 * np.cos(float(theta_40))
    z = L2 * np.sin(float(theta_20)) + L3 * np.sin(float(theta_30)) + L4 * np.sin(float(theta_40))

    x_2 = L2c * np.cos(float(theta_20))
    x_3 = L2 * np.cos(float(theta_20)) + L3c*np.cos(float(theta_30))
    x_4 = L2 * np.cos(float(theta_20)) + L3 * np.cos(float(theta_30)) + L4c * np.cos(float(theta_40))
    x_com = (m2*x_2 + m3*x_3+ m4*x_4) / (m2 + m3 + m4)

    z_2c = L2c * np.sin(float(theta_20))
    z_3c = L2 * np.sin(float(theta_20)) + L3c*np.sin(float(theta_30))
    z_4c = L2 * np.sin(float(theta_20)) + L3 * np.sin(float(theta_30)) + L4c * np.sin(float(theta_40))

    z_com = (m2*z_2c + m3*z_3c + m4*z_4c) / (m2 + m3 + m4)
    theta_P = np.arctan(float(x_com)/float(z_com))

    I_by =  I_2 + I_3 + I_4 +  m2*(z_com-z_2c)**2 +   m3*(z_com-z_3c)**2 +  m4*(z_com-z_4c)**2 

    
    xlist = np.array([x])
    zlist = np.array([z])
    x_comlist = np.array([x_com])
    z_comlist = np.array([z_com])
    l_list = np.array([z_com])
    I_list = np.array([I_by])
    # print(z_com)
    theta_Plist = np.array([theta_P])

    for i in range (0, len(q2list)-1):
        
        theta_2 = np.pi/2 + q2list[i]
        theta_3 = np.pi/2 + q2list[i] + q3list[i]
        theta_4 = np.pi/2 + q2list[i] + q3list[i] + q4list[i]

        x = L2 * np.cos(float(theta_2)) + L3 * np.cos(float(theta_3)) + L4 * np.cos(float(theta_4))
        z = L2 * np.sin(float(theta_2)) + L3 * np.sin(float(theta_3)) + L4 * np.sin(float(theta_4))

        x_2 = L2c * np.cos(float(theta_2))
        x_3 = L2 * np.cos(float(theta_2)) + L3c*np.cos(float(theta_3))
        x_4 = L2 * np.cos(float(theta_2)) + L3 * np.cos(float(theta_3)) + L4c * np.cos(float(theta_4))
        x_com = (m2*x_2 + m3*x_3+ m4*x_4) / (m2 + m3 + m4)

        z_2c = L2c * np.sin(float(theta_2))
        z_3c = L2 * np.sin(float(theta_2)) + L3c*np.sin(float(theta_3))
        z_4c = L2 * np.sin(float(theta_2)) + L3 * np.sin(float(theta_3)) + L4c * np.sin(float(theta_4))

        z_com = (m2*z_2c + m3*z_3c + m4*z_4c) / (m2 + m3 + m4)
        
        l = np.sqrt(float(x_com)**2 + float(z_com)**2)
        theta_P = np.arctan(float(x_com)/float(z_com))
        I_by =  I_2 + I_3 + I_4 +  m2*(z_com-z_2c)**2 +   m3*(z_com-z_3c)**2 +  m4*(z_com-z_4c)**2 

        
        xlist = np.vstack((xlist,x))
        zlist = np.vstack((zlist,z))
        x_comlist = np.vstack((x_comlist,x_com))
        z_comlist = np.vstack((z_comlist,z_com))
        l_list = np.vstack((l_list,l))
        I_list = np.vstack((I_list,I_by))
        theta_Plist = np.vstack((theta_Plist,theta_P))
        
        
    return xlist,zlist, x_comlist, z_comlist, l_list, I_list, theta_Plist



#!/usr/bin/env python

import numpy as np
import sys
from casadi import *
import do_mpc
import sympy as sp
from dmpc import *
import state_equation_pitch as sep

i_mpc = 0

def set_model(init_angle):
    model_type = 'discrete' # either 'discrete' or 'continuous'
    model = do_mpc.model.Model(model_type)

    # States struct (optimization variables):
    # theta_w = model.set_variable(var_type='_x', var_name='theta_w', shape=(1,1))
    theta_1 = model.set_variable(var_type='_x', var_name='theta_1', shape=(1,1))
    theta_2 = model.set_variable(var_type='_x', var_name='theta_2', shape=(1,1))
    dtheta_w = model.set_variable(var_type='_x', var_name='dtheta_w', shape=(1,1))
    dtheta_1 = model.set_variable(var_type='_x', var_name='dtheta_1', shape=(1,1))
    dtheta_2 = model.set_variable(var_type='_x', var_name='dtheta_2', shape=(1,1))


    # Input struct (optimization variables):
    u_w = model.set_variable(var_type='_u', var_name='u_w', shape=(1,1))
    u_b = model.set_variable(var_type='_u', var_name='u_b', shape=(1,1))
    
    # wheel_pos_des = model.set_variable('_tvp', 'wheel_pos_des')
    # wheel_vel_des = model.set_variable('_tvp', 'wheel_vel_des')
    # L_b_pos_des = model.set_variable('_tvp', 'L_b_pos_des')

    # Set expression. These can be used in the cost function, as non-linear constraints
    # or just to monitor another output.
    
    # I_w = 0.004806909
    # I_1 = 0.00614941
    # I_b = 0.572443057472749
    # m_w = 2.292
    # m_1 = 2.786
    # m_b = 19.245
    # L_1 = 0.171
    # L_1c = 0.171
    # L_b = 0.500417013226504
    # r = 0.069
    # g = 9.81
    
    # E_kin_wheel = 1 / 2 * (m_w * (r * dtheta_w)**2 +I_w)
    # E_kin_p1 = 1 / 2 * m_1 * (
    #     (r*dtheta_w + L_1c * dtheta_1 * cos(theta_1))**2 +
    #     (L_1c * dtheta_1 * sin(theta_1))**2) + 1 / 2 * I_1 * dtheta_1**2
    # E_kin_p2 = 1 / 2 * m_b * (
    #     (r*dtheta_w + L_1 * dtheta_1 * cos(theta_1) + L_b * dtheta_2 * cos(theta_2))**2 +
    #     (L_1 * dtheta_1 * sin(theta_1) + L_b * dtheta_2 * sin(theta_2))**
    #     2) + 1 / 2 * I_b * dtheta_1**2

    # E_kin = E_kin_wheel + E_kin_p1 + E_kin_p2

    # E_pot = m_1 * g * L_1c * cos(theta_1) + m_b * g * (L_1 * cos(theta_1) +
    #                             L_b * cos(theta_2))

    # model.set_expression('E_kin', E_kin)
    # model.set_expression('E_pot', E_pot)
    
    # mterm = model.aux['E_kin'] - model.aux['E_pot']
    # lterm = model.aux['E_kin'] - model.aux['E_pot']
    # model.set_expression(expr_name='cost', expr=sum1(theta_1**2 + 5500*(L_b_pos_des-theta_2)**2 + 1.5*(wheel_vel_des-dtheta_w)**2 + dtheta_1**2 + dtheta_2**2))
    model.set_expression(expr_name='cost', expr=sum1(theta_1**2 + theta_2**2 + dtheta_w**2 + dtheta_1**2 + dtheta_2**2))


    Ac, Bc, Cc, Dc = sep.Cal_Pitch_SS()
    
    A, B, C, D = gen_discrete_model_from_cont(Ac, Bc, Cc, Dc, 0.017)
    
    # x_1_next = A[:,0][0]*theta_w + A[:,1][0]*theta_1 + A[:,2][0]*theta_2 + A[:,3][0]*dtheta_w + A[:,4][0]*dtheta_1 + A[:,5][0]*dtheta_2 + B[:,0][0]*u_w + B[:,1][0]*u_b 
    # x_2_next =  A[:,0][1]*theta_w + A[:,1][1]*theta_1 + A[:,2][1]*theta_2 + A[:,3][1]*dtheta_w + A[:,4][1]*dtheta_1 + A[:,5][1]*dtheta_2 + B[:,0][1]*u_w + B[:,1][1]*u_b 
    # x_3_next =  A[:,0][2]*theta_w + A[:,1][2]*theta_1 + A[:,2][2]*theta_2 + A[:,3][2]*dtheta_w + A[:,4][2]*dtheta_1 + A[:,5][2]*dtheta_2 + B[:,0][2]*u_w + B[:,1][2]*u_b
    # x_4_next =  A[:,0][3]*theta_w + A[:,1][3]*theta_1 + A[:,2][3]*theta_2 + A[:,3][3]*dtheta_w + A[:,4][3]*dtheta_1 + A[:,5][3]*dtheta_2 + B[:,0][3]*u_w + B[:,1][3]*u_b
    # x_5_next =  A[:,0][4]*theta_w + A[:,1][4]*theta_1 + A[:,2][4]*theta_2 + A[:,3][4]*dtheta_w + A[:,4][4]*dtheta_1 + A[:,5][4]*dtheta_2 + B[:,0][4]*u_w + B[:,1][4]*u_b
    # x_6_next =  A[:,0][5]*theta_w + A[:,1][5]*theta_1 + A[:,2][5]*theta_2 + A[:,3][5]*dtheta_w + A[:,4][5]*dtheta_1 + A[:,5][5]*dtheta_2 + B[:,0][5]*u_w + B[:,1][5]*u_b 
    
    x_2_next =  A[:,1][1]*theta_1 + A[:,2][1]*theta_2 + A[:,3][1]*dtheta_w + A[:,4][1]*dtheta_1 + A[:,5][1]*dtheta_2 + B[:,0][1]*u_w + B[:,1][1]*u_b 
    x_3_next =  A[:,1][2]*theta_1 + A[:,2][2]*theta_2 + A[:,3][2]*dtheta_w + A[:,4][2]*dtheta_1 + A[:,5][2]*dtheta_2 + B[:,0][2]*u_w + B[:,1][2]*u_b
    x_4_next =  A[:,1][3]*theta_1 + A[:,2][3]*theta_2 + A[:,3][3]*dtheta_w + A[:,4][3]*dtheta_1 + A[:,5][3]*dtheta_2 + B[:,0][3]*u_w + B[:,1][3]*u_b
    x_5_next =  A[:,1][4]*theta_1 + A[:,2][4]*theta_2 + A[:,3][4]*dtheta_w + A[:,4][4]*dtheta_1 + A[:,5][4]*dtheta_2 + B[:,0][4]*u_w + B[:,1][4]*u_b
    x_6_next =  A[:,1][5]*theta_1 + A[:,2][5]*theta_2 + A[:,3][5]*dtheta_w + A[:,4][5]*dtheta_1 + A[:,5][5]*dtheta_2 + B[:,0][5]*u_w + B[:,1][5]*u_b 
    
    # model.set_rhs('theta_w', x_1_next)
    model.set_rhs('theta_1', x_2_next)
    model.set_rhs('theta_2', x_3_next)
    model.set_rhs('dtheta_w', x_4_next)
    model.set_rhs('dtheta_1', x_5_next)
    model.set_rhs('dtheta_2', x_6_next)
    
    # model.set_expression('tvp', wheel_pos_des)
    

    model.setup()
    
    mpc = do_mpc.controller.MPC(model)

    setup_mpc = {
        'n_robust': 0,
        'n_horizon': 80,
        't_step': 0.017,
        'store_full_solution':False,
    }
    # Ts:0.025, H:100, R: 70, t: 0.01
    
    # model.set_expression('tvp', wheel_vel_des)
    # model.set_expression('tvp', L_b_pos_des)
    
    mpc.set_param(**setup_mpc)

    mterm = model.aux['cost']
    lterm = model.aux['cost'] # terminal cost

    mpc.set_objective(mterm=mterm, lterm=lterm)
    # mpc.set_rterm(u_w=500)
    # mpc.set_rterm(u_b=500)
    
    mpc.set_rterm(u_w=4)
    mpc.set_rterm(u_b=100)
    
    tilt_traj = np.load('tilt_traj.npy')
    time_traj = np.load('time.npy')
    traj_vel = np.load('traj_vel.npy')
    time_traj = time_traj.tolist()
    
    # tvp_template = mpc.get_tvp_template()
    
    # # t_switch = 4    # seconds
    # # ind_switch = t_switch // setup_mpc['t_step']
    
    # def tvp_fun(t_ind):
    #     global i_mpc
    #     # tvp_template['_tvp',:, 'wheel_pos_des'] = traj_des[1]

    #     if t_ind == 0:

    #         tvp_template['_tvp',:, 'wheel_vel_des'] = traj_vel[i_mpc]*15000
    #         tvp_template['_tvp',:, 'L_b_pos_des'] = tilt_traj[i_mpc]*0.7
    #         i_mpc += 1
    #     elif 0 < i_mpc < len(time_traj) :
    #         tvp_template['_tvp',:, 'wheel_vel_des'] = traj_vel[i_mpc]*15000
    #         tvp_template['_tvp',:, 'L_b_pos_des'] = tilt_traj[i_mpc]*0.7
    #         i_mpc += 1

    #     # if i_mpc >= 2000:
    #     #     i_mpc = 2000
    #     else: 
    #         tvp_template['_tvp',:, 'wheel_vel_des'] = 0
    #         tvp_template['_tvp',:, 'L_b_pos_des'] = 0
    #         i_mpc += 1
    #     return tvp_template

    # mpc.set_tvp_fun(tvp_fun)

    mpc.setup()
    
    simulator = do_mpc.simulator.Simulator(model)
    estimator = do_mpc.estimator.StateFeedback(model)
    
    simulator.set_param(t_step = 0.017)
    # tvp_template_sim = simulator.get_tvp_template()
    # def tvp_fun_sim(t_ind):
    #     global i_mpc
    #     # tvp_template['_tvp',:, 'wheel_pos_des'] = traj_des[1]

    #     if t_ind == 0:
    #         i_mpc = 0
    #         tvp_template_sim['wheel_vel_des'] = traj_vel[i_mpc]*15000
    #         tvp_template_sim['L_b_pos_des'] = tilt_traj[i_mpc]*0.7
    #         i_mpc += 1
    #     elif 0 < i_mpc < len(time_traj) :
    #         tvp_template_sim['wheel_vel_des'] = traj_vel[i_mpc]*15000
    #         tvp_template_sim['L_b_pos_des'] = tilt_traj[i_mpc]*0.7
    #         i_mpc += 1

    #     # if i_mpc >= 2000:
    #     #     i_mpc = 2000
    #     else: 
    #         tvp_template_sim['wheel_vel_des'] = 0
    #         tvp_template_sim['L_b_pos_des'] = 0
    #         i_mpc += 1
    #     return tvp_template_sim
    # simulator.set_tvp_fun(tvp_fun_sim)
    simulator.setup()
    
    x0 = init_angle
    mpc.x0 = x0
    simulator.x0 = x0
    estimator.x0 = x0
    mpc.set_initial_guess()

    u0 = mpc.make_step(x0)
    
    return mpc, estimator, u0
    

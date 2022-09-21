#!/usr/bin/env python

import sympy as sp
import numpy as np
import control
from EoM import *
from sympy.physics.mechanics import *
import pylab as pl

def Cal_Roll_SS(z):
    I_bx, I_fx, I_fy, m_all, w, l, l_f, f_d, T_gb, g, r, L = sp.symbols('I_bx, I_fx, I_fy, m_all, w, l, l_f, f_d, T_gb, g, r, L')
    K_t, K_e, R, J_m, V, n, f = sp.symbols('K_t, K_e, R, J_m, V, n, f')
    theta_R, theta_gb = dynamicsymbols('theta_R, theta_gb')

    theta_Rd = theta_R.diff()
    theta_Rdd = theta_Rd.diff()
    theta_gbd = theta_gb.diff()
    theta_gbdd = theta_gbd.diff()

    q = sp.Matrix([[theta_R], [theta_gb]])
    qd = q.diff()
    qdd = qd.diff()

    u = sp.Matrix([T_gb])

    L = I_fy * w / 2

    tau = sp.Matrix([[I_bx*theta_Rdd - m_all*g*l*sp.sin(theta_R) + 2*L*theta_gbd*sp.cos(theta_gb)],
                    [I_fx*theta_gbdd -  L*theta_Rd*sp.cos(theta_gb) - T_gb]])
    eq_point = {sp.sin(theta_gb):theta_gb, sp.cos(theta_gb):1,sp.sin(theta_R):theta_R, sp.cos(theta_R):1,theta_gbd**2:0 ,theta_Rd**2:0}

    tau_eq = sp.simplify(tau.subs(eq_point))

    rpm = 5000
    flywheel_ang_vel = (rpm * 2 * np.pi)/60 

    Ml, Cl, Gl, Wl = get_EoM_from_T(tau_eq,qdd,g,u)

    param = {w:flywheel_ang_vel, I_fx:0.006235, I_fy:0.0119107, I_bx:9.448246482,  m_all:22.031, r:0.069, l: z,  g:9.81}

    Mlp = msubs(Ml, param)
    Clp = msubs(Cl, param)
    Glp = msubs(Gl, param)
    Wlp = msubs(Wl, param)

    Mlp_inv = Mlp.inv()
    qdd_rhs_A = Mlp_inv*(-Clp -Glp)
    qdd_rhs_B = Mlp_inv*Wlp*u

    X = q.col_join(qd)
    Xd_A = qd.col_join(qdd_rhs_A)
    Xd_B = qd.col_join(qdd_rhs_B)
    U = u

    A = Xd_A.jacobian(X)
    B = Xd_B.jacobian(U)
    C = X.jacobian(X)
    D = X.jacobian(U)
    
    return A, B, C, D
#!/usr/bin/env python

import sympy as sp
import numpy as np
import control
from EoM import *
from sympy.physics.mechanics import *
import pylab as pl

def Cal_Pitch_SS(z_com):
    
    I_b, I_w, m_b, m_w, r, l_cg, l_f, f_d, f_w, T_w, g = sp.symbols('I_b, I_w, m_b, m_w, r, l_cg, l_f, f_d, f_w, T_w, g')
    theta, phi = dynamicsymbols('theta, phi')

    phid = phi.diff()
    phidd = phid.diff()
    thetad = theta.diff()
    thetadd = thetad.diff()

    q = sp.Matrix([[phi], [theta]])
    qd = q.diff()
    qdd = qd.diff()

    u = sp.Matrix([T_w])

    tau = sp.Matrix([[((m_w+m_b)*r**2 + I_w)*phidd + m_b*r*l_cg*thetadd*sp.cos(theta) - m_b*r*l_cg*thetad**2 * sp.sin(theta) - f_d*r - T_w],
                    [(I_b + m_b*l_cg**2)*thetadd + m_b*r*l_cg*sp.cos(theta)*phidd - m_b*g*l_cg*sp.sin(theta) - f_d*l_f*sp.cos(theta) + T_w]])
    
    zero_point = {f_d:0}
    tau_sum = tau[0] + tau[1]
    tau_sum = sp.simplify(tau_sum.subs(zero_point))
    
    eq_point = {sp.sin(theta):theta, sp.cos(theta):1, thetad**2:0, f_d:0}
    tau_eq = sp.simplify(tau.subs(eq_point))
    Ml, Cl, Gl, Wl = get_EoM_from_T(tau_eq,qdd,g,u)
    
    Ml, Cl, Gl, Wl = get_EoM_from_T(tau_eq,qdd,g,u)

    param = {I_w:0.004806909, I_b:1.9460274096682204, m_w:2.292, m_b:22.477, r:0.069, l_cg: z_com,  g:9.81}

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

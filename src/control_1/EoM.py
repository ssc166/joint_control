import sympy
import pylab as pl
import control
import math
import numpy

def get_EoM_from_T(eom,qdd,g,u):
    # Inertia Matrix, M(q)를 구해주는 부분
    M = eom.jacobian(qdd)

    # Gravity Matrix, G(q) 를 구해주는 부분
    G = sympy.zeros(len(eom),1);
    i = 0;
    for eom_i in eom:
        G_i = [];
        G_i.append(sympy.simplify(sympy.diff(eom_i,g)));
        G[i] = sympy.Matrix(G_i);
        i+=1;
        
    # Input Matrix, u 를 구해주는 부분
    W = sympy.zeros(len(eom),len(u));
    i = 0;
    for eom_i in eom:
        W_i = [];
        W_i.append(sympy.simplify(sympy.diff(eom_i,u.T)));
        W[i,:] = W_i;
        i+=1;
        
    # 원심력 & 코리올리스 행렬, C(q,qd) 를 구해주는 부분
    C = sympy.simplify(eom - M@qdd - G*g - W*u);
    
    return sympy.simplify(M), sympy.simplify(C), sympy.simplify(G*g), sympy.simplify(-W)

def get_Simplified_EoM(eom,q):
    from itertools import product

    t = sympy.symbols('t')

    a = [q.diff(), q.diff()]
    prod = list(product(*a))
    var_combination = []
    for q_i in q:
        var_combination.append(q_i.diff(t,2))
    for prod_i in prod:
        var_combination.append(prod_i[0]*prod_i[1])
        
    result = sympy.Matrix([sympy.expand_trig(eq_i).collect(var_combination, sympy.factor) for eq_i in eom])
    return result

def get_ABCD_from_qdd(Xd,X,u):
    # Get state-space equation
    A = Xd.jacobian(X)
    B = Xd.jacobian(U)
    C = X.jacobian(X)
    D = X.jacobian(U)
            
    return A, B, C, D

def tf_clean(tf, tol=1e-3):
    import copy
    num = copy.deepcopy(tf.num)
    den = copy.deepcopy(tf.den)
    for i_u in range(tf.ninputs):
        for i_y in range(tf.noutputs):
            num[i_y][i_u] = pl.where(abs(num[i_y][i_u]) < tol, pl.zeros(num[i_y][i_u].shape), num[i_y][i_u])
            den[i_y][i_u] = pl.where(abs(den[i_y][i_u]) < tol, pl.zeros(den[i_y][i_u].shape), den[i_y][i_u])
    return control.tf(num,den)

##### 곡선계획법 Trajectory Planning
##### Reference : Modern Robotics Textbook, Chapter 9. Trapezoidal Motion Profile (335 page)
# 사다리꼴 속도 계획법 - 속도최대값과 가속도최대값을 입력으로
def Trapezoidal_Traj_Gen_Given_Vmax_and_Amax(vmax,amax,dt):
    v = vmax
    a = amax
    
    if math.pow(v,2)/a > 1:
        return False
    
    T = (a + math.pow(v,2))/(v*a)
    
    time = 0
    t_save = time
    traj_save = numpy.array([0,0,0])
    while T >= time:
        if time >= 0 and time <= (v/a):
            sddot = a
            sdot = a*time
            s = 0.5*a*math.pow(time,2)
            
        if time > (v/a) and time <= (T-v/a):
            sddot = 0
            sdot = v 
            s = v*time - 0.5*math.pow(v,2)/a
            
        if time > (T-v/a) and time <= T:
            sddot = -a
            sdot = a*(T-time)
            s = (2*a*v*T - 2*math.pow(v,2) - math.pow(a,2)*math.pow((time-T),2))/(2*a)
        
        t_save = numpy.vstack((t_save, time))
        traj_save = numpy.vstack((traj_save, numpy.array([s,sdot,sddot])))
        time += dt

    return t_save, traj_save

##### 곡선계획법 Trajectory Planning
# 사다리꼴 속도 계획법 - 속도최대값과 최종시간을 입력으로
def Trapezoidal_Traj_Gen_Given_Vmax_and_T(vmax,T,dt):
    v = vmax
    
    if v*T > 1 and v*T <= 2:     
        a = math.pow(v,2)/(v*T-1)
    else:
        return False, False            
    
    time = 0
    t_save = time
    traj_save = numpy.array([0,0,0])
    while T >= time:
        if time >= 0 and time <= (v/a):
            sddot = a
            sdot = a*time
            s = 0.5*a*math.pow(time,2)
            
        if time > (v/a) and time <= (T-v/a):
            sddot = 0
            sdot = v 
            s = v*time - 0.5*math.pow(v,2)/a
            
        if time > (T-v/a) and time <= T:
            sddot = -a
            sdot = a*(T-time)
            s = (2*a*v*T - 2*math.pow(v,2) - math.pow(a,2)*math.pow((time-T),2))/(2*a)
        
        t_save = numpy.vstack((t_save, time))
        traj_save = numpy.vstack((traj_save, numpy.array([s,sdot,sddot])))
        time += dt

    return t_save, traj_save

##### 곡선계획법 Trajectory Planning
# 사다리꼴 속도 계획법 - 가속도최대값과 최종시간을 입력으로
def Trapezoidal_Traj_Gen_Given_Amax_and_T(amax,T,dt):
    a = amax
    
    if a*math.pow(T,2) >= 4:     
        v = 0.5*(a*T - math.pow(a,0.5)*math.pow((a*math.pow(T,2)-4),0.5))
    else:
        return False, False            
    
    time = 0
    t_save = time
    traj_save = numpy.array([0,0,0])
    while time < T:
        time += dt
        
        if time >= 0 and time <= (v/a):
            sddot = a
            sdot = a*time
            s = 0.5*a*math.pow(time,2)
            
        if time > (v/a) and time <= (T-v/a):
            sddot = 0
            sdot = v 
            s = v*time - 0.5*math.pow(v,2)/a
            
        if time > (T-v/a) and time <= T:
            if time == T: 
                ssdot = 0
            else: 
                sddot = -a
                
            sdot = a*(T-time)
            s = (2*a*v*T - 2*math.pow(v,2) - math.pow(a,2)*math.pow((time-T),2))/(2*a)
    
        t_save = numpy.vstack((t_save, time))
        traj_save = numpy.vstack((traj_save, numpy.array([s,sdot,sddot])))
       
    return t_save, traj_save
#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Exercises 8.12, 8.17 from Kane 1985."""

from __future__ import division
from sympy import expand, solve, symbols, trigsimp
from sympy.physics.mechanics import ReferenceFrame, Point
from sympy.physics.mechanics import inertia, RigidBody
from sympy.physics.mechanics import dot, dynamicsymbols
from util import msprint, partial_velocities
from util import generalized_active_forces, generalized_inertia_forces

print("\nEx8.12")
## --- Declare symbols ---
q1 = dynamicsymbols('q1')
u1, u2, u3 = dynamicsymbols('u1:4')
u_prime, R, M, g, e, f, theta = symbols('u\' R, M, g, e, f, theta')
Q1, Q2, Q3 = symbols('Q1, Q2 Q3')
F3 = symbols('F3')

# --- Reference Frames ---
F = ReferenceFrame('F')
P = F.orientnew('P', 'axis', [-theta, F.y])
A = P.orientnew('A', 'axis', [q1, P.x])
A.set_ang_vel(F, u1*A.x + u3*A.z)

## --- define points D, S*, Q on frame A and their velocities ---
pD = Point('D')
pD.set_vel(A, 0)
# u3 will not change v_D_F since wheels are still assumed to roll without slip.
pD.set_vel(F, u2 * A.y)

pS_star = pD.locatenew('S*', e*A.y)
pQ = pD.locatenew('Q', f*A.y - R*A.x)
for p in [pS_star, pQ]:
    p.set_vel(A, 0)
    p.v2pt_theory(pD, F, A)

## --- define partial velocities ---
partials = partial_velocities([pD, pS_star, pQ], [u1, u2, u3],
                              F, express_frame=A)

forces = [(pS_star, -M*g*F.x), (pQ, Q1*A.x + Q2*A.y + Q3*A.z)]
torques = []
Fr, _ = generalized_active_forces(partials, forces + torques, uaux=[u3])
print("Generalized active forces:")
for i, f in enumerate(Fr, 1):
    print("F{0} = {1}".format(i, msprint(f)))

friction = -u_prime*Q1*(pQ.vel(F).normalize().express(A)).subs(u3, 0)
Q_map = dict(zip([Q2, Q3], [dot(friction, x) for x in [A.y, A.z]]))
Q_map[Q1] = trigsimp(solve(F3 - Fr[-1].subs(Q_map), Q1)[0])
print('')
for x in [Q1, Q2, Q3]:
    print('{0} = {1}'.format(x, msprint(Q_map[x])))

print("\nEx8.17")
### --- define new symbols ---
a, b, mA, mB, IA, J, K, t = symbols('a b mA mB IA J K t')
IA22, IA23, IA33 = symbols('IA22 IA23 IA33')
q2, q3 = dynamicsymbols('q2 q3')
q2d, q3d = dynamicsymbols('q2 q3', level=1)

# define frames for wheels
B = A.orientnew('B', 'axis', [q2, A.z])
C = A.orientnew('C', 'axis', [q3, A.z])

# masscenters of bodies A, B, C
pA_star = pD.locatenew('A*', a*A.y)
pB_star = pD.locatenew('B*', b*A.z)
pC_star = pD.locatenew('C*', -b*A.z)
for p in [pA_star, pB_star, pC_star]:
    p.set_vel(A, 0)
    p.v2pt_theory(pD, F, A)

# points of B, C touching the plane P
pB_hat = pB_star.locatenew('B^', -R*A.x)
pC_hat = pC_star.locatenew('C^', -R*A.x)
pB_hat.set_vel(B, 0)
pC_hat.set_vel(C, 0)
pB_hat.v2pt_theory(pB_star, F, B)
pC_hat.v2pt_theory(pC_star, F, C)

# the velocities of B^, C^ are zero since B, C are assumed to roll without slip
#kde = [dot(p.vel(F), b) for b in A for p in [pB_hat, pC_hat]]
kde = [dot(p.vel(F), A.y) for p in [pB_hat, pC_hat]]
kde_map = solve(kde, [q2d, q3d])
# need to add q2'', q3'' terms manually since subs does not replace
# Derivative(q(t), t, t) with Derivative(Derivative(q(t), t))
for k, v in kde_map.items():
    kde_map[k.diff(t)] = v.diff(t)

# inertias of bodies A, B, C
# IA22, IA23, IA33 are not specified in the problem statement, but are
# necessary to define an inertia object. Although the values of
# IA22, IA23, IA33 are not known in terms of the variables given in the
# problem statement, they do not appear in the general inertia terms.
inertia_A = inertia(A, IA, IA22, IA33, 0, IA23, 0)
inertia_B = inertia(B, K, K, J)
inertia_C = inertia(C, K, K, J)

# define the rigid bodies A, B, C
rbA = RigidBody('rbA', pA_star, A, mA, (inertia_A, pA_star))
rbB = RigidBody('rbB', pB_star, B, mB, (inertia_B, pB_star))
rbC = RigidBody('rbC', pC_star, C, mB, (inertia_C, pC_star))
bodies = [rbA, rbB, rbC]

# create new partial velocities since we define new points A*, B*, C* and
# new frames B, C
system = []
for b in bodies:
    system += [b.masscenter, b.frame]
partials2 = partial_velocities(system, [u1, u2, u3], F,
                               kde_map, express_frame=A)

Fr_star, _ = generalized_inertia_forces(partials2, bodies, kde_map, uaux=[u3])
print("Generalized inertia forces:")
for i, f in enumerate(Fr_star, 1):
    print("F{0} = {1}".format(i, msprint(expand(trigsimp(f)))))

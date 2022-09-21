#!/usr/bin/env python

import sympy as sp
import numpy as np
import control
from EoM import *
from sympy.physics.mechanics import *
import pylab as pl

def Cal_Pitch_SS():
    
    A = np.array([[   0, 0 , 0 , 1   ,0  ,  0],
                [   0, 0, 0, 0, 1, 0 ],
                [   0, 0, 0, 0, 0, 1 ],
                [   0, -856.80315448, -29.13267444, 0, 0, 0 ],
                [   0, 582.09299048, -184.55345608, 0, 0, 0],
                [   0, -72.19423545, 77.48185211, 0, 0, 0]])

    B = np.array([[   0, 0 ],
                [   0, 0],
                [   0, 0],
                [   83.14078137, -37.08185261],
                [   -38.93409828, 5.47973882],
                [   1.64509166, 3.08194318]])
    
    # A = np.array([[   0, 0 , 0 , 1   ,0  ,  0],
    #             [   0, 0, 0, 0, 1, 0 ],
    #             [   0, 0, 0, 0, 0, 1 ],
    #             [   0, -856.80315448, -29.13267444, 0, 0, 0 ],
    #             [   0, 582.09299048, -184.55345608, 0, 0, 0],
    #             [   0, -72.19423545, 77.48185211, 0, 0, 0]])

    # B = np.array([[   0, 0 ],
    #             [   0, 0],
    #             [   0, 0],
    #             [   83.14078137, 22.8752832],
    #             [   -38.93409828, -17.70390674],
    #             [   1.64509166, 2.77358082]])
    
    C = np.array([[1,0,0,0,0,0],
                  [0,1,0,0,0,0],
                  [0,0,1,0,0,0],
                  [0,0,0,1,0,0],
                  [0,0,0,0,1,0],
                  [0,0,0,0,0,1]])
    
    D = np.array([[0,0],
                [0,0],
                [0,0],
                [0,0],
                [0,0],
                [0,0]])
    
    return A, B, C, D

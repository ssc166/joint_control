#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import random

class RTplot():
    def __init__(self):
        self.X = []
        self.Y = []
        self.C = []
        self.upCount = 0
        self.fig = plt.figure(figsize=(12,6))
        self.ax = plt.subplot()
        plt.autoscale(enable=True)


    def aniFunc(self, i):
        self.addPoint([random.randrange(1,100),random.randrange(1,100)])
        self.ax.cla()
        self.ax.scatter(self.X, self.Y, c=self.C)

    def addPoint(self, list):
        if self.upCount == 100:
            self.clearPoint()
            self.upCount = 0
        self.X.append(list[0])
        self.Y.append(list[1])
        self.C.append("#%06x" % random.randint(0, 0xFFFFFF))
        self.upCount += 1

    def clearPoint(self):
        self.X.clear()
        self.Y.clear()
        self.C.clear()
   
    def start(self):
        self.ani = FuncAnimation(self.fig, self.aniFunc, interval=1)
        plt.show()

if __name__ == "__main__":

    plot = RTplot()
    plot.start()
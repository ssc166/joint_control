#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from itertools import count




index = count()
x_index = []
x_list = []


def animate(i):
    x_index.append(next(index))

    plt.cla()
    plt.plot(x_index[:len(x_list)], x_list)

i = 0
while True:
    x_list.append(i)

    ani=animation.FuncAnimation(plt.gcf(), animate, interval = 10)
    plt.tight_layout()
    plt.show()
    i += 1

# sineani.save('sine.mp4', fps=10)

plt. show( )
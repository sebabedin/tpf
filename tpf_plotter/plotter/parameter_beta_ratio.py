'''
Created on May 8, 2024

@author: seb
'''

import os
import csv
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

def points_distance(p1, p2):
    x1 = p1[0]
    y1 = p1[1]
    x2 = p1[0]
    y2 = p1[1]
    x = x1 - x2
    y = y1 - y2
    d = math.sqrt(x*x + y*y)
    return d

def triangle_area

triangleHLength = std::abs(signedTriangleArea(neigh[midIndex], neigh.front(), neigh.back())) / triangleBLength;

if __name__ == '__main__':
    
    angles = [x for x in range(10, 89, 10)]
    radius = 1.
    
    fig, axs = plt.subplots(2, len(angles))
    for ax, angle in zip(axs[0], angles):
        ax.set_aspect('equal', 'box')
        theta = angle * math.pi / 180
        x1 = math.cos(theta)
        y1 = math.sin(theta)
        x2 = math.cos(-theta)
        y2 = math.sin(-theta)
        x = [x1, 0, x2]
        y = [y1, 0, y2]
        ax.plot(x, y, 'r')
        circle = Circle((0, 0), 1, fill=False)
        ax.add_patch(circle)
    
    plt.grid(True)
    plt.show()
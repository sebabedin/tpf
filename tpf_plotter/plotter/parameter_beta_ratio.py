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
    x2 = p2[0]
    y2 = p2[1]
    x = x1 - x2
    y = y1 - y2
    d = math.sqrt(x*x + y*y)
    return d

def signed_triangle_area(p0, p1, p2):
    return ((p2[1] - p1[1]) * p0[0] - (p2[0] - p1[0]) * p0[1] + p2[0] * p1[1] - p2[1] * p1[0])


def plot_bratio():
    angles = [x for x in range(10, 90, 10)]
    radius = 0.5
    bratios = [0.5, 1., 2.5, 4.]
    
    fig, axs = plt.subplots(len(bratios), len(angles), sharex=True, sharey=True)
    fig.suptitle(f'Evaluación de la apertura angular de la esquina\nen fución de Beta con Radio = {radius}')
    
    for i, bratio, ax_rows in zip(range(0, len(bratios)), bratios, axs):
        for j, ax, angle in zip(range(0, len(angles)), ax_rows, angles):
            ax.set_aspect('equal', 'box')
            if (len(bratios))-1 == i:
                ax.set_xlabel(f'{2*angle}°')
            
            theta = angle * math.pi / 180
            x1 = math.cos(theta)
            y1 = math.sin(theta)
            x2 = math.cos(-theta)
            y2 = math.sin(-theta)
            
            p0 = (0, 0)
            p1 = (x1, y1)
            p2 = (x2, y2)
            
            triangle_b = points_distance(p1, p2)
            triangle_h = math.fabs(signed_triangle_area(p0, p1, p2)) / triangle_b
            bRatio = 2.5
            
            valid = True
            if (triangle_b < (radius / bratio)) or (triangle_h < (radius / bratio)):
                valid = False
            
            x = [x1, 0, x2]
            y = [y1, 0, y2]
            color = 'g' if valid else 'r'
            ax.plot(x, y, color)
            circle = Circle((0, 0), 1, fill=False, linestyle=':')
            ax.add_patch(circle)
            ax.set_xlim((0., radius))
            ax.set_ylim((-radius, radius))
            ax.set_xticks([])
            ax.set_xticks([], minor=True)
            ax.set_yticks([])
            ax.set_yticks([], minor=True)
            
            if 0 == j:
                ax.set_ylabel(f'Beta = {bratio}')
            ax.grid(True)

def plot_bratio_vs_radius():
    angles = [x for x in range(10, 90, 10)]
    radii = [0.1, 0.5, 1., 1.5]
    bratio = 2.5
    
    fig, axs = plt.subplots(len(radii), len(angles), sharex=True, sharey=True)
    fig.suptitle(f'Evaluación de la apertura angular de la esquina\nen fución del Radio con Beta = {bratio}')
    
    for i, radius, ax_rows in zip(range(0, len(radii)), radii, axs):
        for j, ax, angle in zip(range(0, len(angles)), ax_rows, angles):
            ax.set_aspect('equal', 'box')
            if (len(radii))-1 == i:
                ax.set_xlabel(f'{2*angle}°')
            
            theta = angle * math.pi / 180
            x1 = math.cos(theta)
            y1 = math.sin(theta)
            x2 = math.cos(-theta)
            y2 = math.sin(-theta)
            
            p0 = (0, 0)
            p1 = (x1, y1)
            p2 = (x2, y2)
            
            triangle_b = points_distance(p1, p2)
            triangle_h = math.fabs(signed_triangle_area(p0, p1, p2)) / triangle_b
            
            valid = True
            if (triangle_b < (radius / bratio)) or (triangle_h < (radius / bratio)):
                valid = False
            
            x = [x1, 0, x2]
            y = [y1, 0, y2]
            color = 'g' if valid else 'r'
            ax.plot(x, y, color)
            circle = Circle((0, 0), 1, fill=False, linestyle=':')
            ax.add_patch(circle)
            ax.set_xlim((0., 1.))
            ax.set_ylim((-1., 1.))
            ax.set_xticks([])
            ax.set_xticks([], minor=True)
            ax.set_yticks([])
            ax.set_yticks([], minor=True)
            
            if 0 == j:
                ax.set_ylabel(f'Radio = {radius}')
            
            ax.grid(True)

if __name__ == '__main__':
    
    plot_bratio()
    plot_bratio_vs_radius()
    
    plt.show()
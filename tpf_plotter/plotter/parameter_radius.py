'''
Created on May 8, 2024

@author: seb
'''

import os
import csv
import math
import numpy as np
import matplotlib.pyplot as plt

def get_radius(rho, alpha, beta):
    radius = alpha * math.exp(beta * rho);
    if radius >= rho:
        radius = rho * 0.8
    return radius

if __name__ == '__main__':
    
    alpha_default = 0.1
    beta_default = 0.07
    alphas = [0.1, 0.05, 0.2]
    betas = [0.07, 0.05, 0.1]
    
    x = np.linspace(0.1, 30., 1000)
    
    fig, axs = plt.subplots(2, 1, sharex=True)
    fig.suptitle('Radio en función de los parámetros alpha y beta')
    ax = axs[0]
    for p in alphas:
        a = p
        b = beta_default
        y = [get_radius(x, a, b) for x in x]
        label = f'alpha: {a}, beta: {b}'
        ax.plot(x, y, label=label)
    ax.grid(True)
    ax.legend()
    ax.set_ylabel('Radio [m]')
    
    ax = axs[1]
    for p in betas:
        a = alpha_default
        b = p
        y = [get_radius(x, a, b) for x in x]
        label = f'alpha: {a}, beta: {b}'
        ax.plot(x, y, label=label)
    ax.grid(True)
    ax.legend()
    ax.set_ylabel('Radio [m]')
    ax.set_xlabel('Distancia [m]')
    
    plt.show()

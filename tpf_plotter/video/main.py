'''
Created on May 5, 2024

@author: seb
'''

import os
import shutil
import subprocess
import time
import tracemalloc
import matplotlib.pyplot as plt
from lib import *

def plot_node1(ax, node):
    node.plot_max_range(ax)
    node.plot_pose(ax)
    node.plot_scan(ax, point_color='b')
    
def plot_node2(ax, node):
    node.plot_pose(ax)
    node.plot_scan(ax, point_color='r')
    
def plot_node1_keypoints(ax, node):
    for kp in node.kps:
        kp_points = node.points[kp]
        
        center = (kp_points.x, kp_points.y)
        radius = 0.5
        circle = Circle(center, radius, fill=False, linestyle='-', edgecolor='b')
        ax.add_patch(circle)

def plot_node2_keypoints(ax, node, kps_filter):
    kps = []
    for kp_filter in kps_filter:
        kps.append(node.kps[kp_filter])
    
    for kp in kps:
        kp_points = node.points[kp]
        center = (kp_points.x, kp_points.y)
        radius = 0.5
        circle = Circle(center, radius, fill=False, linestyle='-', edgecolor='r')
        ax.add_patch(circle)

def plot_node1_match_node2(ax, node1, node2):
    kps_pair_index = node1.matches[node2.id]
    for kp_pair_index in kps_pair_index:
        kp1_index = kp_pair_index[0]
        kp2_index = kp_pair_index[1]
        kp1_point = node1.points[node1.kps[kp1_index]]
        kp2_point = node2.points[node2.kps[kp2_index]]
        x = [kp1_point.x, kp2_point.x]
        y = [kp1_point.y, kp2_point.y]
        ax.plot(x, y, '-g')
    
    # for kp in kps:
    #     kp_points = node.points[kp]
    #     center = (kp_points.x, kp_points.y)
    #     radius = 0.5
    #     circle = Circle(center, radius, fill=False, linestyle='-', edgecolor='r')
    #     ax.add_patch(circle)

def plot_keypoints(ax, node1):
    plot_node1_keypoints(ax, node1)
    for node2_id in node1.matches:
        node2 = exp.nodes[node2_id]
        kps_filter = [x[1] for x in node1.matches[node2_id]]
        plot_node2_keypoints(ax, node2, kps_filter)
        plot_node1_match_node2(ax, node1, node2)
    
def plot_match(ax, exp, node):
    node = exp.nodes[node_id]
    
    for node_match_id in node.matches:
        node_match = exp.nodes[node_match_id]
        
        plot_node2(ax, node_match)
    
    plot_node1(ax, node)
    plot_keypoints(ax, node)
    
def plot_global_match(ax, exp, node):
    ax.set_aspect('equal', 'box')
    ax.grid(True)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('Vista global')
    
    points = []
    for node in exp.nodes.values():
        points += node.points
    
    x = [p.x for p in points]
    y = [p.y for p in points]
    
    x_min = min(x)
    x_max = max(x)
    dx = x_max - x_min
    x_mean = (x_max + x_min) / 2
    
    y_min = min(y)
    y_max = max(y)
    dy = y_max - y_min
    y_mean = (y_max + y_min) / 2
    
    d_max = max([dx, dy])
    
    ax.set_xlim((x_mean - d_max/2 - d_max*0.1, x_mean + d_max/2 + d_max*0.1))
    ax.set_ylim((y_mean - d_max/2 - d_max*0.1, y_mean + d_max/2 + d_max*0.1))
    ax.xaxis.set_major_locator(plticker.MultipleLocator(5.00))
    
    plot_match(ax, exp, node)
    
def plot_local_match(ax, exp, node):
    ax.grid(True)
    ax.set_aspect('equal', 'box')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('Vista local')
    
    x_mean = node.pose.x
    y_mean = node.pose.y
    d_max = 20.
    
    ax.set_xlim((x_mean - d_max/2 - d_max*0.1, x_mean + d_max/2 + d_max*0.1))
    ax.set_ylim((y_mean - d_max/2 - d_max*0.1, y_mean + d_max/2 + d_max*0.1))
    ax.xaxis.set_major_locator(plticker.MultipleLocator(2.50))
    ax.yaxis.set_major_locator(plticker.MultipleLocator(2.50))
    
    plot_match(ax, exp, node)
    
def plot_frame(exp, node_id):
    node = exp.nodes[node_id]
    fig, axs = plt.subplots(1, 2)
    n_matches = len(node.matches)
    title = 'Experimento: %s, kp Th: %d, Nodo: %4d, Cantidad de asociaciones: %3d' % (exp.name(), exp.kp_th, node_id, n_matches)
    fig.suptitle(title)
    
    fig.tight_layout()
    plt.gcf().set_size_inches(16, 9)
    plot_global_match(axs[0], exp, node)
    plot_local_match(axs[1], exp, node)
    return fig

def create_directory(directory_path):
    try:
        # Use os.makedirs to create the directory and its parents if they don't exist
        os.makedirs(directory_path)
        # print(f"Directory '{directory_path}' created successfully.")
    except FileExistsError:
        print(f"Directory already exists: {directory_path}")
    except Exception as e:
        print(f"Error creating directory: {e}")

def remove_directory(directory_path):
    try:
        # Use shutil.rmtree to remove the directory and its contents
        shutil.rmtree(directory_path)
        # print(f"Directory '{directory_path}' and its contents removed successfully.")
    except FileNotFoundError:
        print(f"Directory not found: {directory_path}")
    except Exception as e:
        print(f"Error removing directory: {e}")

def create_video(exp_directory_path, file_name):
    cwd = os.getcwd()
    os.chdir(exp_directory_path)
    subprocess.call(['ffmpeg', '-framerate', '24', '-i', 'node_%04d.png', '-r', '30', '-pix_fmt', 'yuv420p', file_name])
    os.chdir(cwd)

if __name__ == '__main__':
    root = os.path.join('.', '..', 'data')
    
    files_paths = [
        (os.path.join(root, 'keypoint_01.csv'), os.path.join(root, 'match_01.csv')),
        # (os.path.join(root, 'keypoint_03.csv'), os.path.join(root, 'match_03.csv')),
        # (os.path.join(root, 'keypoint_02.csv'), os.path.join(root, 'match_02.csv')),
        # (os.path.join(root, 'keypoint_04.csv'), os.path.join(root, 'match_04.csv')),
        ]
    
    # kp_ths = [1, 2, 3, 4, 5]
    kp_ths = [1]
    
    for kp_th in kp_ths:
        print(f'kp_th: {kp_th}')
        
        experiments = []
        for idx, file_path in zip(range(0, len(files_paths)), files_paths):
            experiments.append(Experiment(idx, file_path[0], file_path[1], id_th=150, kp_th=kp_th))
        
        #      0123456789   0123456789   0123456789   0123456789
        print("experiment | nodes      | matches    | ")
        for exp in experiments:
            print("%10d | %10d | %10d |" % (exp.index, len(exp.nodes), len(exp.matches)))
        
        if False:
            for exp in experiments:
                print(f'{exp.name()}')
                for node in exp.nodes.values():
                    n_matches = len(node.matches)
                    if 0 < n_matches:
                        print(f'node {node.id}, matches: {n_matches}')
        
        if False:
            tracemalloc.start()
            for exp in experiments:
                exp_directory_path = os.path.join('.', 'out', f'exp_{exp.name()}_kpth_{exp.kp_th}')
                # remove_directory(exp_directory_path)
                create_directory(exp_directory_path)
                
                node_ids = list(exp.nodes.keys())
                for node_id in node_ids[]:
                    file_path = os.path.join(exp_directory_path, 'node_%04d.png' % (node_id))
                    fig = plot_frame(exp, node_id)
                    # plt.show()
                    plt.savefig(file_path, dpi=300)
                    fig.clear()
                    plt.close(fig)
                    print(f'plot frame for node: {node_id}/{len(node_ids)}')
                    print(tracemalloc.get_traced_memory())
            tracemalloc.stop()
        
        if False:
            for exp in experiments:
                exp_directory_path = os.path.join('.', 'out', f'exp_{exp.name()}_kpth_{exp.kp_th}')
                file_name = f'exp_{exp.name()}_kpth_{exp.kp_th}.mp4'
                create_video(exp_directory_path, file_name)


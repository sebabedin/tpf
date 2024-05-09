'''
Created on May 5, 2024

@author: seb
'''

import os
import csv
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

class Topic(object):

    def __init__(self, file_path, msg_class) -> None:
        self.msgs = []
        with open(file_path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            for row in csv_reader:
                self.msgs.append(msg_class(row))

# class TopicNodeKeypoint(Topic):
#
#     def __init__(self, file_path) -> None:
#         super().__init__(file_path, MsgNodeKeypoint)

class MsgNodeKeypoint(Topic):

    def __init__(self, row) -> None:
        self.id = int(row[0])
        self.x = float(row[1])
        self.y = float(row[2])
        self.yaw = float(row[3])
        self.kp_len = int(row[4])
        self.kp = [int(x) for x in row[5:5+self.kp_len]]
        self.ranges = [float(x) for x in row[5+self.kp_len:]]

# class TopicNodeMatch(object):
#
#     def __init__(self, file_path) -> None:
#         super().__init__(file_path, MsgNodeMatch)

class MsgNodeMatch(object):

    def __init__(self, row) -> None:
        self.id1 = int(row[0])
        self.id2 = int(row[1])
        self.matchs = int(row[2])
        self.kp1 = [int(x) for x in row[3:3+self.matchs]]
        self.kp2 = [int(x) for x in row[3+self.matchs:]]
        if not len(self.kp1) == len(self.kp2):
            raise Exception()

def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

def points_distance(p1, p2):
    x = p1.x - p2.x
    y = p1.y - p2.y
    return math.sqrt(x*x + y*y)

class Point2D(object):

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.plot_color = 'r'
    
    def plot(self, ax):
        format_ = f'.{self.plot_color}'
        ax.plot([self.x], [self.y], format_, markersize=0.1)
        
class Pose2D(object):

    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
        
        self.t = Point2D(self.x, self.y)
    
    def plot(self, ax):
        point = Point2D(self.x, self.y)
        point.plot_color = 'r'
        point.plot(ax)
        
        dx, dy = pol2cart(0.05, self.yaw)
        ax.arrow(self.x, self.y, dx, dy, width=0.1)

class LIDARScan(object):
    angle_min = 0.
    angle_max = 0.
    angle_inc = 0.
    n_beams = 0

    def __init__(self, ranges):
        self.angle_min = 0
        self.fov = 2 * math.pi
        self.n_beams = 360
        self.angle_inc = (self.fov) / (self.n_beams)
        self.angle_max = (self.n_beams - 1) * self.angle_inc
        self.angles = np.linspace(self.angle_min, self.angle_max, num=self.n_beams)
        self.ranges = ranges
        
    def get_points(self, pose):
        points = []
        for rho, theta in zip(self.ranges, self.angles):
            if math.isinf(rho):
                rho = 0.
                
            x, y = pol2cart(rho, theta + pose.yaw)
            # x, y = pol2cart(rho, theta)
            x += pose.x
            y += pose.y
            point = Point2D(x, y)
            points.append(point)
        return points
    
    # def plot(self, ax, format='ob'):
    #     ax.plot([self.x], [self.y], format)
        
# class Scan(object):
#
#     def __init__(self, ranges):
#         # self.lidar = lidar
#         # self.pose = pose
#         self.ranges = ranges
#
#     def generate_points(self, lidar, pose):
#
#     def plot(self, ax, format='ob'):
#         ax.plot([self.x], [self.y], format)
        
class Node(object):

    def __init__(self, id_, pose, scan, kps):
        self.id= id_
        self.pose = pose
        self.scan = scan
        self.kps = kps
        self.match = {}
        
        self.points = self.scan.get_points(self.pose)
        # self.plot_points_limit 
    
    # @classmethod
    # def new_from_msg(cls, msg, lidar_class):
    #     pose = Pose2D(msg.x, msg.y, msg.yaw)
    #     scan = LIDARScan(msg.ranges)
    #     return cls(msg.id, pose, scan, msg.kp)
    
    def plot(self, ax):
        ax.text(self.pose.x, self.pose.y, f'{self.id}')
        self.pose.plot(ax)
        
        for point in self.points:
            point.plot_color = 'b'
            point.plot(ax)

class Match(object):

    def __init__(self, node1, node2, kps1, kps2):
        self.node1 = node1
        self.node2 = node2
        self.id_d = self.node1.id - self.node2.id 
        
        self.kps = []
        for kp1, kp2 in zip(kps1, kps2):
            if 0 <= kp2:
                self.kps.append((kp1, kp2))
        
        self.d = points_distance(node1.pose.t, node2.pose.t)
        theta = node1.pose.yaw - node2.pose.yaw
        
        theta = theta - (math.ceil((theta + math.pi)/(2*math.pi))-1)*2*math.pi
        self.theta = theta

    def plot(self, ax):
        x1 = self.node1.pose.x
        x2 = self.node2.pose.x
        y1 = self.node1.pose.y
        y2 = self.node2.pose.y
        ax.plot([x1, x2], [y1, y2], 'b', linewidth=0.5)

def graph_nodes_keypoints_histogram(nodes):
    fig, ax = plt.subplots(1, 1)
    x = []
    for node in nodes.values():
        x.append(len(node.kps))
    max_len = max(x)
    print(f'max kps size: {max_len}')
    x_bins = [x for x in range(0, max_len+1, 1)]
    counts, bins = np.histogram(x, bins=x_bins)
    plt.stairs(counts, bins)
    plt.grid(True)
    
def graph_nodes_keypoints_cumulative_distributions(nodes):
    # https://matplotlib.org/stable/gallery/statistics/histogram_cumulative.html
    fig, ax = plt.subplots(1, 1)
    x = []
    for node in nodes.values():
        x.append(len(node.kps))
    n_bins = max(x)
    # n, bins, patches = 
    ax.hist(x, n_bins, density=True, histtype="step", cumulative=True, label="Cumulative histogram")
    plt.grid(True)

def graph_nodes_match_keypoints_histogram(nodes, topic_node_match):
    n_match_nodes = {}
    n_match_nodes_all = {}
    for node in nodes.values():
        n_match_nodes[node.id] = 0
        n_match_nodes_all[node.id] = 0
    
    for msg in topic_node_match.msgs:
        kp2_valid = 0
        for kp2 in msg.kp2:
            if 0 <= kp2:
                kp2_valid += 1
        
        n_match_nodes[msg.id1] = kp2_valid
        n_match_nodes_all[msg.id1] = len(msg.kp2)
    
    x = [x for x in n_match_nodes.values()]
    y = [y for y in n_match_nodes_all.values()]
    
    fig, ax = plt.subplots(1, 1)
    
    max_len = max([max(x), max(y)])
    # print(f'max kps size: {max_len}')
    x_bins = [x for x in range(0, max_len+1, 1)]
    counts, bins = np.histogram(x, bins=x_bins)
    plt.stairs(counts, bins)
    
    # max_len = max(y)
    x_bins = [x for x in range(0, max_len+1, 1)]
    counts, bins = np.histogram(y, bins=x_bins)
    plt.stairs(counts, bins)
    
    plt.grid(True)
    
def graph_nodes_match_histogram(matchs, kps_th=0):
    
    nodes = {}
    for match_ in matchs:
        if kps_th <= len(match_.kps):
            nodes[match_.node1.id] = 0
        
    for match_ in matchs:
        if kps_th <= len(match_.kps):
            nodes[match_.node1.id] += 1
    
    x = [x for x in nodes.values()]
    
    fig, ax = plt.subplots(1, 1)
    
    max_len = max(x)
    x_bins = [x for x in range(0, max_len+1, 1)]
    counts, bins = np.histogram(x, bins=x_bins)
    plt.stairs(counts, bins)
    
    plt.grid(True)

def graph_distances_match_histogram(nodes, matchs, kps_edges):
    n_match_nodes = {}
    for node in nodes.values():
        n_match_nodes[node.id] = 0
    
    n_kps = max([len(x.kps) for x in matchs])
    print(f'n_kps: {n_kps}')
    
    fig, ax = plt.subplots(1, 1)
    for n_kp in kps_edges:
        d = []
        theta = []
        for match_ in matchs:
            if n_kp == len(match_.kps):
                d.append(match_.d)
                theta.append(match_.theta)
        
        ax.plot(theta, d, 'o', markersize=0.5)
    
    plt.grid(True)

def graph_distances_match_colormesh(nodes, matchs, n_kp=1):
    n_match_nodes = {}
    for node in nodes.values():
        n_match_nodes[node.id] = 0
    
    fig, ax = plt.subplots(1, 1)
    d = []
    theta = []
    for match_ in matchs:
        if n_kp == len(match_.kps):
            d.append(match_.d)
            theta.append(match_.theta)
        
    angles = np.linspace(-math.pi, math.pi, num=100)
    distances = np.linspace(0., 16., num=100)
    
    H, xedges, yedges = np.histogram2d(theta, d, bins=[angles, distances])
    H = H.T
    ax.pcolormesh(xedges, yedges, H)
    
    plt.grid(True)

def match_filter(matchs, id_d_th):
    new_matchs = []
    for match_ in matchs:
        id_d = match_.node1.id - match_.node2.id
        if id_d_th <= id_d:
            new_matchs.append(match_)
    return new_matchs

def graph_nodes_map(nodes):
    fig, ax = plt.subplots(1, 1)
    ax.set_aspect('equal', 'box')
    
    for node in nodes.values():
        node.pose.plot(ax)
    plt.grid(True)
    
def graph_matchs_map(matchs, nodes={}, kp_th=0, path=[], id_d_th=0.):
    fig, ax = plt.subplots(1, 1)
    ax.set_aspect('equal', 'box')
    
    # for node in nodes.values():
    #     node.plot(ax)
    
    if 0 < len(path):
        x = []
        y = []
        for node in path:
            x.append(node.pose.x)
            y.append(node.pose.y)
        ax.plot(x, y, 'r:', linewidth=0.5)
    
    for match_ in matchs:
        if kp_th <= len(match_.kps):
            if id_d_th <= match_.id_d:
                match_.plot(ax)
    
    plt.grid(True)

if __name__ == '__main__':
    topic_file_path = os.path.join('.', '..', 'data', 'node_keypoint_01.csv')
    topic_node_keypoint = Topic(topic_file_path, MsgNodeKeypoint)
    
    topic_file_path = os.path.join('.', '..', 'data', 'node_match_01.csv')
    topic_node_match = Topic(topic_file_path, MsgNodeMatch)
    
    nodes = {}
    for msg in topic_node_keypoint.msgs:
        pose = Pose2D(msg.x, msg.y, msg.yaw)
        scan = LIDARScan(msg.ranges)
        node = Node(msg.id, pose, scan, msg.kp)
        nodes[node.id] = node
    
    matchs = []
    for msg in topic_node_match.msgs:
        node1 = nodes[msg.id1]
        kps = [(x, y) for x, y in zip(msg.kp1, msg.kp2)]
        node1.match[msg.id2] = kps
        
        node2 = nodes[msg.id2]
        kps = [(x, y) for x, y in zip(msg.kp2, msg.kp1)]
        node2.match[msg.id1] = kps
        
        match_ = Match(nodes[msg.id1], nodes[msg.id2], msg.kp1, msg.kp2)
        if 0 < len(match_.kps):
            matchs.append(match_)
    
    print(f'nodes: {len(nodes)}')
    print(f'matchs: {len(matchs)}')
    
    # Histogrma cantidad de kypoints
    # graph_nodes_keypoints_histogram(nodes)
    # graph_nodes_keypoints_cumulative_distributions(nodes)
    
    # graph_nodes_match_keypoints_histogram(nodes, topic_node_match)
    # graph_nodes_match_histogram(matchs)
    # graph_nodes_match_histogram(matchs, 3)
    
    graph_distances_match_histogram(nodes, matchs, [1, 3, 5, 7])
    # graph_distances_match_histogram(nodes, matchs, [3, 5, 7])
    # graph_distances_match_histogram(nodes, matchs, [5, 7])
    # graph_distances_match_colormesh(nodes, matchs, n_kp=1)
    # graph_distances_match_colormesh(nodes, matchs, n_kp=3)
    # graph_distances_match_colormesh(nodes, matchs, n_kp=5)
    
    matchs_filtered = match_filter(matchs, 150)
    # print(f'len(matchs)={len(matchs)}')
    # print(f'len(matchs_filtered)={len(matchs_filtered)}') 
    # graph_distances_match_histogram(nodes, matchs_filtered, [1, 3, 5, 7])
    # graph_distances_match_histogram(nodes, matchs_filtered, [3, 5, 7])
    # graph_distances_match_histogram(nodes, matchs_filtered, [5, 7])
    
    # graph_nodes_map(nodes)
    # graph_matchs_map(matchs, nodes, 0)
    # graph_matchs_map(matchs, nodes, 3)
    # graph_matchs_map(matchs, kp_th=3, path=nodes.values(), id_d_th=150)
    # graph_matchs_map(matchs, kp_th=3, path=nodes.values(), id_d_th=0)
    # graph_matchs_map(matchs, nodes, 7)
    
    plt.show()


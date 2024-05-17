
import os
import csv
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import matplotlib.ticker as plticker

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
        ax.plot([self.x], [self.y], format_, markersize=1.0)
        
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
        self.matches = {}
        self.points = self.scan.get_points(self.pose)
    
    def add_match(self, node_id, kps_index):
        self.matches[node_id] = kps_index
    
    def plot_max_range(self, ax):
        center = (self.pose.x, self.pose.y)
        radius = 10.
        circle = Circle(center, radius, fill=False, linestyle=':')
        ax.add_patch(circle)
    
    def plot_id(self, ax):
        ax.text(self.pose.x, self.pose.y, f'{self.id}')
    
    def plot_pose(self, ax):
        self.pose.plot(ax)
    
    def plot_scan(self, ax, point_color = 'b'):
        for point in self.points:
            point.plot_color = point_color
            point.plot(ax)
            
    def plot(self, ax):
        self.plot_pose(ax)
        self.plot_scan(ax)

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

class Experiment(object):
    
    def __init__(self, index, keypoint_file, match_file, id_th=150, kp_th=0):
        self.id_th = id_th
        self.kp_th = kp_th
        self.index = index
        self.topic_node_keypoint = Topic(keypoint_file, MsgNodeKeypoint)
        self.topic_node_match = Topic(match_file, MsgNodeMatch)
        
        self.nodes = {}
        for msg in self.topic_node_keypoint.msgs:
            pose = Pose2D(msg.x, msg.y, msg.yaw)
            scan = LIDARScan(msg.ranges)
            node = Node(msg.id, pose, scan, msg.kp)
            self.nodes[node.id] = node
        
        self.matches = []
        for msg in self.topic_node_match.msgs:
            match_ = Match(self.nodes[msg.id1], self.nodes[msg.id2], msg.kp1, msg.kp2)
            
            if match_.id_d < id_th:
                continue
            
            if len(match_.kps) < kp_th:
                continue
            
            self.matches.append(match_)
            match_.node1.add_match(match_.node2.id, match_.kps)
    
    def name(self):
        return f'{self.index+1}'

def match_filter(matchs, id_d_th):
    new_matchs = []
    for match_ in matchs:
        id_d = match_.node1.id - match_.node2.id
        if id_d_th <= id_d:
            new_matchs.append(match_)
    return new_matchs

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
        ax.plot([x1, x2], [y1, y2], 'b', linewidth=1.5)

def graph_nodes_keypoints_histogram(nodes):
    fig, ax = plt.subplots(1, 1)
    # fig.suptitle(f'Keypoints por nodos')
    x = []
    for node in nodes.values():
        x.append(len(node.kps))
    max_len = max(x)
    print(f'max kps size: {max_len}')
    x_bins = [x for x in range(0, max_len+1, 1)]
    counts, bins = np.histogram(x, bins=x_bins)
    # plt.stairs(counts, bins)
    plt.hist(bins[:-1], bins, weights=counts)
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
    # plt.stairs(counts, bins)
    plt.hist(bins[:-1], bins, weights=counts)
    
    # max_len = max(y)
    x_bins = [x for x in range(0, max_len+1, 1)]
    counts, bins = np.histogram(y, bins=x_bins)
    # plt.stairs(counts, bins)
    plt.hist(bins[:-1], bins, weights=counts)
    
    plt.grid(True)
    

def experiments_graph_nodes_match_histogram(exps, kps_edges=[0]):
    fig, axs = get_figure(len(kps_edges), len(exps))
    
    for i, axr, kp_edge in zip(range(0, len(kps_edges)), axs, kps_edges):
        for j, ax, exp in zip(range(0, len(exps)), axr, exps):
            graph_nodes_match_histogram(ax, exp, kp_edge)

def graph_nodes_match_histogram(ax, exp, kp_edge=0):
    matchs = exp.matchs_filtered
    
    nodes = {}
    # for match_ in matchs:
    #     if kp_edge <= len(match_.kps):
    #         nodes[match_.node1.id] = 0
        
    for match_ in matchs:
        if kp_edge <= len(match_.kps):
            if match_.node1.id in nodes:
                nodes[match_.node1.id] += 1
            else:
                nodes[match_.node1.id] = 1
    
    x = [x for x in nodes.values()]
    
    # fig, ax = plt.subplots(1, 1)
    
    max_len = max(x)
    x_bins = [x for x in range(0, max_len+1, 1)]
    # counts, bins = np.histogram(x, bins=x_bins)
    counts, bins = np.histogram(x)
    ax.stairs(counts, bins)
    
    # plt.grid(True)

def experiments_graph_distances_match_histogram(experiments, kps_edges=[0]):
    fig, axs = get_figure(len(kps_edges), len(experiments), sharex=True, sharey=False)
    
    d_max_table = []
    for i in range(0, len(kps_edges)):
        d_max_table.append([0 for x in range(0, len(experiments))])
    
    for i, axr, kp_edge in zip(range(0, len(kps_edges)), axs, kps_edges):
        for j, ax, exp in zip(range(0, len(experiments)), axr, experiments):
            if 0 == i:
                ax.set_title(f'Exp. {exp.name()}')
            if 0 == j:
                ax.set_ylabel(f'min kps = {kp_edge}\nDistancia [m]')
            else:
                ax.yaxis.set_tick_params(labelleft=False)
            graph_distances_match_histogram(ax, exp, kp_edge, i, j, d_max_table)
            ax.set_xticks([-180, -90, 0, 90, 180])
    
    for j in range(0, len(experiments)):
        axs[0][j].set_yticks([0, 5, 10, 15, 20])
        axs[1][j].set_yticks([0, 5, 10, 15, 20])
    
    for i in range(0, len(d_max_table)):
        d_max = max(d_max_table[i])
        for j in range(0, len(d_max_table[i])):
            axs[i][j].set_ylim((-d_max*0.1, d_max*1.1))
    
    for j in range(0, len(experiments)):
        axs[len(kps_edges)-1][j].set_xlabel(f'Rotación [°]')
    
        # for j in range(0, len(d_max_table[i])):
    # print(d_max_table)
    
        # ax.set_ylim((0, max(d)))
    
    out_file = os.path.join('.', 'out', f'matchs_hist_kp.pdf')
    fig.tight_layout()
    plt.gcf().set_size_inches(10, 10)
    plt.savefig(out_file, dpi=300, format='pdf')

def graph_distances_match_histogram(ax, exp, kp_edge, i, j, d_max_table):
    nodes = exp.nodes
    matchs = exp.matchs
    n_match_nodes = {}
    for node in nodes.values():
        n_match_nodes[node.id] = 0
    
    n_kps = max([len(x.kps) for x in matchs])
    print(f'n_kps: {n_kps}')
    
    d = []
    theta = []
    for match_ in matchs:
        if kp_edge <= len(match_.kps):
            d.append(match_.d)
            theta.append(match_.theta * 180 / math.pi)
    
    ax.plot(theta, d, 'o', markersize=1.0)
    
    ax.set_xlim((-180*1.1, 180*1.1))
    # ax.set_ylim((0, max(d)))
    ax.grid(True)
    
    d_max_table[i][j] = max(d)

def experiments_graph_distances_match_colormesh(experiments, kps_edges=[0]):
    fig, axs = get_figure(len(kps_edges), len(experiments), sharex=True, sharey=True)
    
    for i, axr, kp_edge in zip(range(0, len(kps_edges)), axs, kps_edges):
        for j, ax, exp in zip(range(0, len(experiments)), axr, experiments):
            if 0 == i:
                ax.set_title(f'Exp. {exp.name()}')
            if 0 == j:
                ax.set_ylabel(f'kp = {kp_edge}')
            
            graph_distances_match_colormesh(ax, exp, kp_edge)
    
    out_file = os.path.join('.', 'out', f'matchs_colormesh.pdf')
    fig.tight_layout()
    plt.gcf().set_size_inches(10, 6.5)
    plt.savefig(out_file, dpi=300, format='pdf')

def graph_distances_match_colormesh(ax, exp, kp_edge=1):
    nodes = exp.nodes
    matchs = exp.matchs_filtered

    n_match_nodes = {}
    for node in nodes.values():
        n_match_nodes[node.id] = 0
    
    # fig, ax = plt.subplots(1, 1)
    d = []
    theta = []
    for match_ in matchs:
        if kp_edge == len(match_.kps):
            d.append(match_.d)
            theta.append(match_.theta)
        
    angles = np.linspace(-math.pi, math.pi, num=100)
    distances = np.linspace(0., 16., num=100)
    
    H, xedges, yedges = np.histogram2d(theta, d, bins=[angles, distances])
    H = H.T
    
    cmap = plt.colormaps["plasma"]
    cmap = cmap.with_extremes(bad=cmap(0))
    # ax.pcolormesh(xedges, yedges, H, cmap=cmap, norm="log", rasterized=True)
    ax.pcolormesh(xedges, yedges, H, cmap=cmap, rasterized=True)
    
    # plt.grid(True)

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
    
def get_figure(nrow=1, ncol=1, sharex=False, sharey=False):
    n_ax = nrow*ncol
    if 1 == n_ax:
        fig, ax = plt.subplots(nrow, ncol, sharex=sharex, sharey=sharey)
        return fig, [[ax]]
    elif 1 == nrow:
        fig, axs = plt.subplots(nrow, ncol, sharex=sharex, sharey=sharey)
        return fig, [axs]
    elif 1 == ncol:
        fig, axs = plt.subplots(nrow, ncol, sharex=sharex, sharey=sharey)
        axs_ = [[x] for x in axs]
        return fig, axs_
    else:
        fig, axs = plt.subplots(nrow, ncol, sharex=sharex, sharey=sharey)
        return fig, axs

def experiments_graph_matchs_map(experiments, kp_ths=[0], id_d_th=0.):
    fig, axs = get_figure(len(kp_ths), len(experiments))
    for i, axr, kp_th in zip(range(0, len(kp_ths)), axs, kp_ths):
        for j, ax, exp in zip(range(0, len(experiments)), axr, experiments):
            if 0 == i:
                ax.set_title(f'Exp. {exp.name()}')
            if 0 == j:
                ax.set_ylabel(f'min kps = {kp_th}\nY [m]')
                
            graph_matchs_map(ax, exp, kp_th, id_d_th)
            # ax.set_xlabel('[m]')
    
    # i_last = len(kp_ths) - 1
    for i in range(0, len(kp_ths)):
        for j in range(0, len(experiments)):
            if not i == (len(kp_ths) - 1):
                axs[i][j].set_xticklabels([])
            else:
                ax.set_xlabel(f'X [m]')
    
    # for i in range(0, len(kp_ths)):
    #     if not i == (len(kp_ths) - 1):
    #             # axs[i][j].set_xticks([])
    #             # axs[i][j].set_xticks([])
    #             # axs[i][j].set_xticks([], minor=True)
    #             axs[i][j].grid(True)
        # axs[i][0].set_yticks([-15, -10, -5, 0, 5, 10, 15, 20])
        # axs[1][j].set_yticks([0, 5, 10, 15, 20])
    
    # for i in range(0, len(kp_ths)):
    #     for j, exp in zip(range(0, len(experiments)), experiments):
    #         x = []
    #         y = []
    #         path = exp.nodes.values()
    #         for node in path:
    #             x.append(node.pose.x)
    #             y.append(node.pose.y)
    #         xlim = (min(x)*0.9, max(x)*1.1)
    #         ylim = (min(y)*0.9, max(y)*1.1)
    #         print(xlim)
    #         print(ylim)
    #
    #         axs[i][j].set_xlim(xlim)
    #         axs[i][j].set_ylim(ylim)
            

    
    out_file = os.path.join('.', 'out', f'paths_matchs_vs_kp.pdf')
    fig.tight_layout()
    plt.gcf().set_size_inches(10, 10)
    plt.savefig(out_file, dpi=300, format='pdf')
    
def graph_matchs_map(ax, exp, kp_th=0, id_d_th=0.):
    matchs = exp.matchs
    nodes = exp.nodes
    path = exp.nodes.values()
    # fig, ax = plt.subplots(1, 1)
    ax.set_aspect('equal', 'box')
    ax.set_box_aspect(1)
    
    # for node in nodes.values():
    #     node.plot(ax)
    
    if 0 < len(path):
        x = []
        y = []
        for node in path:
            x.append(node.pose.x)
            y.append(node.pose.y)
        ax.plot(x, y, 'r:', linewidth=1.0)
    
    for match_ in matchs:
        if kp_th <= len(match_.kps):
            if id_d_th <= match_.id_d:
                match_.plot(ax)
    
    # loc = plticker.MultipleLocator(base=1.0) # this locator puts ticks at regular intervals
    # ax.xaxis.set_major_locator(loc)
    
    # ax.set_xticks([])
    # ax.set_xticks([], minor=True)
    # ax.set_yticks([])
    # ax.set_yticks([], minor=True)
    
    # ax.xaxis.set_major_locator(plticker.IndexLocator(base=5., offset=0.))
    ax.xaxis.set_major_locator(plticker.MultipleLocator(5.00))
    ax.grid(True)
    # ax.tick_params(
    #     axis='x',          # changes apply to the x-axis
    #     which='both',      # both major and minor ticks are affected
    #     bottom=False,      # ticks along the bottom edge are off
    #     top=False,         # ticks along the top edge are off
    #     labelbottom=False) # labels along the bottom edge are off
    

class Experiment(object):
    
    def __init__(self, index, keypoint_file, match_file, id_distance=150):
        self.index = index
        # topic_file_path = os.path.join('.', '..', 'data', 'keypoint_01.csv')
        self.topic_node_keypoint = Topic(keypoint_file, MsgNodeKeypoint)
        
        # topic_file_path = os.path.join('.', '..', 'data', 'match_01.csv')
        self.topic_node_match = Topic(match_file, MsgNodeMatch)
        
        self.nodes = {}
        for msg in self.topic_node_keypoint.msgs:
            pose = Pose2D(msg.x, msg.y, msg.yaw)
            scan = LIDARScan(msg.ranges)
            node = Node(msg.id, pose, scan, msg.kp)
            self.nodes[node.id] = node
        
        self.matchs = []
        for msg in self.topic_node_match.msgs:
            node1 = self.nodes[msg.id1]
            kps = [(x, y) for x, y in zip(msg.kp1, msg.kp2)]
            node1.match[msg.id2] = kps
            
            node2 = self.nodes[msg.id2]
            kps = [(x, y) for x, y in zip(msg.kp2, msg.kp1)]
            node2.match[msg.id1] = kps
            
            match_ = Match(self.nodes[msg.id1], self.nodes[msg.id2], msg.kp1, msg.kp2)
            if 0 < len(match_.kps):
                self.matchs.append(match_)
        
        self.matchs_filtered = match_filter(self.matchs, id_distance)
    
    def name(self):
        return f'{self.index+1}'

def experiments_table_nodes_match_mean(exps, kps_edges=[0]):
    kp_nodes = []
    kp_match_means = []
    for i, kp_edge in zip(range(0, len(kps_edges)), kps_edges):
        exp_nodes = []
        exp_match_means = []
        for j, exp in zip(range(0, len(exps)), exps):
            nodes, match_mean = nodes_match_mean(exp, kp_edge)
            exp_nodes.append(nodes)
            exp_match_means.append(match_mean)
        kp_nodes.append(exp_nodes)
        kp_match_means.append(exp_match_means)
    
    print('')
    print('Nodos candidatos con matchs')
    print(f'{"".rjust(10)} | ', end='')
    for exp in exps:
        exp_name = f'Exp. {exp.name()}'
        print(f'{exp_name.rjust(10)} | ', end='')
    print('')
    
    for i, kp_edge, exp_nodes in zip(range(0, len(kps_edges)), kps_edges, kp_nodes):
        label = 'min kp %3d' % (kp_edge)
        print(f'{label.rjust(10)} | ',  end='')
        for j, exp, nodes in zip(range(0, len(exps)), exps, exp_nodes):
            value = '%10d' % (nodes)
            print(f'{value.rjust(10)} | ', end='')
        print('')
    
    print('')
    print('Promedio de match por nodo')
    print(f'{"".rjust(10)} | ', end='')
    for exp in exps:
        exp_name = f'Exp. {exp.name()}'
        print(f'{exp_name.rjust(10)} | ', end='')
    print('')
    
    for i, kp_edge, exp_match_means in zip(range(0, len(kps_edges)), kps_edges, kp_match_means):
        label = 'min kp %3d' % (kp_edge)
        print(f'{label.rjust(10)} | ',  end='')
        for j, exp, match_means in zip(range(0, len(exps)), exps, exp_match_means):
            value = '%10f' % (match_means)
            print(f'{value.rjust(10)} | ', end='')
        print('')
    
            #     status = True if 'True' == str(row['status']) else False
            # print(f' {str(index).rjust(3)} | {str(status).ljust(6)} | {test_id}')
    
    # print("experiment | nodes      | matchs     | filtered")
    # for exp in experiments:
    #     print("%10d | %10d | %10d | %10d" % (exp.index, len(exp.nodes), len(exp.matchs), len(exp.matchs_filtered)))

def nodes_match_mean(exp, kp_edge=0):
    matchs = exp.matchs_filtered
    
    nodes = {}
    for match_ in matchs:
        if kp_edge <= len(match_.kps):
            if match_.node1.id in nodes:
                nodes[match_.node1.id] += 1
            else:
                nodes[match_.node1.id] = 1
    
    x = [x for x in nodes.values()]
    sum_x = sum(x)
    mean_x = sum_x / len(x)
    return len(nodes), mean_x

if __name__ == '__main__':
    # topic_file_path = os.path.join('.', '..', 'data', 'keypoint_01.csv')
    # topic_node_keypoint = Topic(topic_file_path, MsgNodeKeypoint)
    #
    # topic_file_path = os.path.join('.', '..', 'data', 'match_01.csv')
    # topic_node_match = Topic(topic_file_path, MsgNodeMatch)
    #
    # nodes = {}
    # for msg in topic_node_keypoint.msgs:
    #     pose = Pose2D(msg.x, msg.y, msg.yaw)
    #     scan = LIDARScan(msg.ranges)
    #     node = Node(msg.id, pose, scan, msg.kp)
    #     nodes[node.id] = node
    #
    # matchs = []
    # for msg in topic_node_match.msgs:
    #     node1 = nodes[msg.id1]
    #     kps = [(x, y) for x, y in zip(msg.kp1, msg.kp2)]
    #     node1.match[msg.id2] = kps
    #
    #     node2 = nodes[msg.id2]
    #     kps = [(x, y) for x, y in zip(msg.kp2, msg.kp1)]
    #     node2.match[msg.id1] = kps
    #
    #     match_ = Match(nodes[msg.id1], nodes[msg.id2], msg.kp1, msg.kp2)
    #     if 0 < len(match_.kps):sharex
    #         matchs.append(match_)
    
    root = os.path.join('.', '..', 'data')
    
    files_paths = [
        (os.path.join(root, 'keypoint_01.csv'), os.path.join(root, 'match_01.csv')),
        (os.path.join(root, 'keypoint_03.csv'), os.path.join(root, 'match_03.csv')),
        (os.path.join(root, 'keypoint_02.csv'), os.path.join(root, 'match_02.csv')),
        (os.path.join(root, 'keypoint_04.csv'), os.path.join(root, 'match_04.csv')),
        ]
    
    # match_files = [
    #     os.path.join(root, 'match_01.csv'),
    #     os.path.join(root, 'match_02.csv'),
    #     os.path.join(root, 'match_03.csv'),
    #     os.path.join(root, 'match_04.csv'),
    #     ]
    
    experiments = []
    for idx, file_path in zip(range(0, len(files_paths)), files_paths):
        experiments.append(Experiment(idx, file_path[0], file_path[1]))
    
    #      0123456789   0123456789   0123456789   0123456789
    print("experiment | nodes      | matchs     | ")
    for exp in experiments:
        print("%10d | %10d | %10d |" % (exp.index, len(exp.nodes), len(exp.matchs_filtered)))
    
    # Histogrma cantidad de kypoints
    # graph_nodes_keypoints_histogram(nodes)
    # graph_nodes_keypoints_cumulative_distributions(nodes)
    
    # graph_nodes_match_keypoints_histogram(nodes, topic_node_match)
    # graph_nodes_match_histogram(matchs)
    # graph_nodes_match_histogram(matchs, 3)
    # experiments_graph_nodes_match_histogram(experiments, [2, 3, 4, 5])
    # experiments_table_nodes_match_mean(experiments, [1, 2, 3, 4, 5])
    
    # graph_distances_match_histogram(nodes, matchs, [1, 3, 5, 7])
    # graph_distances_match_histogram(nodes, matchs, [3, 5, 7])
    # graph_distances_match_histogram(nodes, matchs, [5, 7])
    # graph_distances_match_colormesh(nodes, matchs, n_kp=1)
    # graph_distances_match_colormesh(nodes, matchs, n_kp=3)
    # experiments_graph_distances_match_colormesh(experiments, [2, 3, 4])
    
    # experiments_graph_distances_match_histogram(experiments, [2, 3, 4, 5])
    
    # graph_nodes_map(nodes)
    # graph_matchs_map(matchs, nodes, 0)
    # graph_matchs_map(matchs, nodes, 3)
    
    experiments_graph_matchs_map(experiments, kp_ths=[2, 3, 4, 5], id_d_th=150)
    
    plt.show()


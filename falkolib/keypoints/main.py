'''
Created on Apr 19, 2024

@author: seb
'''

import os
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np
import json
import subprocess
import math

import topic_laser_scan
import nms

def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

class ExperimentSetup(object):
    
    def __init__(self, scan_msg):
        self.msg = scan_msg
        self.MinExtractionRange = 0.5
        self.MaxExtractionRange = 10.0
        self.EnableSubbeam = False
        self.NMSRadius = 0.30
        self.NeighA = 0.30
        self.NeighB = 0.10
        self.BRatio = 2.5
        self.GridSectors = 16.0
        self.MinScoreTh = 50.0
        self.ScanAngleMin = scan_msg.angle_min
        self.ScanFOV = scan_msg.range_fov
        self.ScanNumBeams = scan_msg.n_beams
        self.ScanRanges = scan_msg.ranges

    def to_dict(self):
        return {
            "MinExtractionRange": self.MinExtractionRange,
            "MaxExtractionRange": self.MaxExtractionRange,
            "EnableSubbeam": self.EnableSubbeam,
            "NMSRadius": self.NMSRadius,
            "NeighA": self.NeighA,
            "NeighB": self.NeighB,
            "BRatio": self.BRatio,
            "GridSectors": self.GridSectors,
            "MinScoreTh": self.MinScoreTh,
            "ScanAngleMin": self.ScanAngleMin,
            "ScanFOV": self.ScanFOV,
            "ScanNumBeams": self.ScanNumBeams,
            "ScanRanges": self.ScanRanges,
        }

    def to_json(self, file_name):
        json_object = json.dumps(self.to_dict(), indent=4)
        with open(file_name, "w") as outfile:
            outfile.write(json_object)

class Point2D(object):

    def __init__(self, x_, y_):
        self.x = x_
        self.y = y_
        self.rho, self.phi = cart2pol(self.x, self.y)
    
    def plot(self, ax, format='ob'):
        ax.plot([self.x], [self.y], format)

class Keypoint(object):

    def __init__(self, index, orientation, radius, point):
        self.index = index
        self.orientation = orientation
        self.radius = radius
        self.point = point
    
    def plot(self, ax):
        self.point.plot(ax, 'xr')
        
        center = (self.point.x, self.point.y)
        radius = self.radius
        circle = Circle(center, radius, fill=False)
        ax.add_patch(circle)
        
        dx, dy = pol2cart(1., self.orientation)
        ax.arrow(self.point.x, self.point.y, dx, dy, width=0.2)
        
        # ax.axline((0, 0), (self.point.x, self.point.y)),

class Experiment(object):
    
    def __init__(self, setup, kpdata, debug_data):
        self.setup = setup
        self._generate_points()
        
        self.kpdata = kpdata
        self._generate_keypoints()
        
        self.debug = debug_data
        self._process_scores()
    
    def _generate_points(self):
        min_angle = self.setup.ScanAngleMin
        max_angle = self.setup.ScanAngleMin + self.setup.ScanFOV
        n_beams = self.setup.ScanNumBeams
        ranges = self.setup.ScanRanges
        self.angles = np.linspace(min_angle, max_angle, num=n_beams)
        
        self.points = []
        for rho, theta in zip(ranges, self.angles):
            x, y = pol2cart(rho, theta)
            point = Point2D(x, y)
            self.points.append(point)
            
        self.index = [x for x in range(0, len(self.points))]

    def _generate_keypoints(self):
        keypoints_list = self.kpdata['keypoints']
        self.keypoints = []
        for keypoints_info in keypoints_list:
            index = keypoints_info['index']
            point = self.points[index]
            keypoint = Keypoint(keypoints_info['index'], keypoints_info['orientation'], keypoints_info['radius'], point)
            self.keypoints.append(keypoint)

    def _process_scores(self):
        beams = self.debug['beams']
    
        self.scores_pre = []
        for beam in beams:
            self.scores_pre.append(beam['scores'])
    
        scores_info_filtered = self.debug['score_filtered']
        self.scores_post = []
        for score_info in scores_info_filtered:
            self.scores_post.append(score_info['score'])

    def _init_kp_graph(self):
        self.kp_fig, self.kp_ax = plt.subplots()
        
    def clean_kp_graph(self):
        self.kp_ax.clear()
    
    def update_kp_graph(self):
        self._init_kp_graph()
        self._plot_kp_graph(self.kp_fig, self.kp_ax)
        
    def _plot_kp_graph(self, fig, ax):
        ax.set_aspect('equal', 'box')
    
        for point, idx in zip(self.points, self.index):
            point.plot(ax)
            ax.text(point.x, point.y, f'{idx}')
    
        for keypoint in self.keypoints:
            keypoint.plot(ax)
    
        center = (0, 0)
        radius = self.setup.MinExtractionRange
        circle = Circle(center, radius, fill=False)
        ax.add_patch(circle)
        
        center = (0, 0)
        radius = self.setup.MaxExtractionRange
        circle = Circle(center, radius, fill=False)
        ax.add_patch(circle)
        
        center = (0, 0)
        radius = self.setup.msg.range_min
        circle = Circle(center, radius, fill=False, linestyle=':')
        ax.add_patch(circle)
        
        center = (0, 0)
        radius = self.setup.msg.range_max
        circle = Circle(center, radius, fill=False, linestyle=':')
        ax.add_patch(circle)
    
        ax.grid()
        fig.tight_layout()
        # fig.canvas.draw() 
        # fig.canvas.flush_events()

    def plot_radius(self):
        fig, ax = plt.subplots()
        
        x = np.linspace(self.setup.MinExtractionRange, self.setup.MaxExtractionRange, num=100)
        y = self.setup.NeighA * np.exp( self.setup.NeighB * x)
        
        ax.plot(x, y)
        
        plt.grid()
        fig.tight_layout()

    def _init_score_graph(self):
        self.score_fig, self.score_ax = plt.subplots(2, 1, sharex=True)
        
    def clean_score_graph(self):
        self.score_ax.clear()
    
    def update_score_graph(self):
        self._init_score_graph()
        self._plot_score_graph(self.score_fig, self.score_ax)

    def _plot_score_graph(self, fig, ax):
        ax[0].plot(self.index, self.scores_pre, 'o-')
        ax[0].grid()
        ax[1].plot(self.index, self.scores_post, 'o-')
        minval = self.debug['scoreMax'] * self.setup.MinScoreTh / 100.
        ax[1].axline((0, minval), (1, minval))
        ax[1].grid()

    # def plot_points(points):
    #     fig, ax = plt.subplots()
    #     ax.set_aspect('equal', 'box')
    #
    #     index = [x for x in range(0, len(points))]
    #     for point, idx in zip(points, index):
    #         point.plot(ax)
    #         # ax.annotate(f'{idx}', xy=(point.x, point.y))
    #         ax.text(point.x, point.y, f'{idx}')
    #
    #     plt.grid()
    #     # plt.show()
    #     fig.tight_layout()

def main():
    
    scan_file_path = os.path.join('.', '..', 'data', 'scan.csv')
    topic = topic_laser_scan.LaserScanTopic(scan_file_path)
    
    # plt.ion()
    print(len(topic.msgs))
    
    for msg in topic.msgs[250:251]: 
    # msg = topic.msgs[0]
        
        setup = ExperimentSetup(msg)
        
        setup_file = os.path.join('.', 'keypoint_setup.json')
        setup.to_json(setup_file)
        
        
        bin_path = os.path.join('/home/seb/ros2ws/rbtpf/src/openslam_falkolib/bin/falkobsc_keypoint')
        output = subprocess.check_output([bin_path])
        output = output.decode('UTF-8')
        output = output.replace(",]", "]")
        output = output.replace(",\n}", "}")
        output = output.replace(",\n]", "]")
        
        output_file = os.path.join('.', 'debug.json')
        with open(output_file, 'w') as f:
            f.write(output)
            
        debug_file = os.path.join('.', 'debug.json')
        with open(debug_file, 'r') as file:
            debug_data = json.load(file)
        
        keypoints_file = os.path.join('.', 'keypoint.json')
        with open(keypoints_file, 'r') as file:
            keypoint_data = json.load(file)
        
        experiment = Experiment(setup, keypoint_data, debug_data)
        # experiment.clean_kp_graph()
        experiment.update_kp_graph()
        experiment.update_score_graph()
        plt.show()
        
        scores = experiment.scores_post
        ranges = experiment.setup.ScanRanges
        angle_inc = experiment.setup.ScanFOV / experiment.setup.ScanNumBeams
        ibeg = 0
        iend = experiment.setup.ScanNumBeams
        radius = experiment.setup.NMSRadius
        minvalue = experiment.debug['scoreMax'] * experiment.setup.MinScoreTh / 100.0
        peaks = nms.NMSKeypoint(scores, ranges, angle_inc, ibeg, iend, radius, minvalue)
        print(peaks)
        
        
        # experiment.plot_radius()
        
        # points = get_points(keypoint_data)
        # keypoints_ = get_keypoints(keypoint_data, points)
        #
        # process_scores(debug_data)
        #
        # plot_keypoints(keypoints_, points, keypoint_data, debug_data)
        
    
if __name__ == '__main__':
    main()

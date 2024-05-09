# import os
# import sys
# import json
# import subprocess
# import numpy as np
# import matplotlib.pyplot as plt
# import math
# import laser
# import falko_cpp

# header:
#   stamp:
#     sec: 6
#     nanosec: 250000000
#   frame_id: robot_laser
# angle_min: -2.094395160675049
# angle_max:  2.090304374694824
# angle_increment: 0.004090615548193455
# time_increment: 0.0
# scan_time: 0.0
# range_min: 0.0
# range_max: 4.0
# ranges:
# intensities: []

import os
import csv
import math

class LaserScanTopic(object):

    def __init__(self, file_path) -> None:
        self.msgs = []
        with open(file_path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            for row in csv_reader:
                self.msgs.append(LaserScanMsg(row))

class LaserScanMsg(object):

    def __init__(self, row) -> None:
        self.sec = int(row[0])
        self.nanosec = int(row[1])
        self.frame_id = row[2]
        self.angle_min = float(row[3])
        self.angle_max = float(row[4])
        self.angle_increment = float(row[5])
        # self.angle_max = self.angle_min + self.angle_increment*(self.n_beams-1)
        self.time_increment = float(row[6])
        self.scan_time = float(row[7])
        self.range_min = float(row[8])
        self.range_max = float(row[9])
        self.range_fov = self.angle_max - self.angle_min
        self.n_beams = int(math.floor(self.range_fov / self.angle_increment) + 1)
        ranges = []
        for idx in range(10, 10 + self.n_beams):
            beam_range = float(row[idx])
            if self.range_max < beam_range:
                beam_range = 0.0
            ranges.append(beam_range)
        self.ranges = ranges
        

'''
Created on May 17, 2024

@author: seb
'''

import os
import subprocess


if __name__ == '__main__':
    exp_directory_path = os.path.join('.', 'out', f'exp_1')
    # file_path = os.path.join(exp_directory_path, 'node_%04d.png' % (node_id))
    os.chdir(exp_directory_path)
    subprocess.call([
        'ffmpeg', '-framerate', '16', '-i', 'node_%04d.png', '-r', '30', '-pix_fmt', 'yuv420p',
        'video_exp_2.mp4'
    ])
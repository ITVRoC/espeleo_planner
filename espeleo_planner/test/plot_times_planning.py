#!/usr/bin/env python
# -*- coding: utf-8 -*-

import statistics
import numpy as np
import matplotlib.pyplot as plt
import traceback
import numpy as np
import pcl
import pcl.pcl_visualization
import time
import os
import json
import sys
import math
import argparse
import scipy.stats
from mpl_toolkits.mplot3d import Axes3D


path_data_raw = {
    1193: {
            'time': [0.58, 0.60, 0.67],
            'distance': 2.14
        },
        
    2226: {
            'time': [1.29, 1.3, 1.13],
            'distance': 6.25
        },
    2928: {
            'time': [1.83, 3.04, 1.94],
            'distance': 14.96
        },
    4776: {
            'time': [2.09, 2.22, 2.52],
            'distance': 4.43
        },
    5339: {
            'time': [2.99, 2.88, 2.53],
            'distance': 5.18
        },
    5588: {
            'time': [3,03, 2.68, 3.13],
            'distance': 10.79
        },
    6965: {
            'time': [3.4, 3.57, 3.69],
            'distance': 1.74
        },
    7011: {
            'time': [4.05, 4.42, 4.08],
            'distance': 16.94
        },
    7280: {
            'time': [4.61, 4.1, 3.84],
            'distance': 3.88
        },
    7210: {
            'time': [4.31, 4.25, 4.50],
            'distance': 10.25
        },
    7702: {
            'time': [4.08, 6.14, 4.3],
            'distance': 3.06
        },
    8007: {
            'time': [4.37, 4.57, 4.42],
            'distance': 2.49
        },
    8518: {
            'time': [4.58, 4.9, 4.52],
            'distance': 2.42
        },
    9163: {
            'time': [5.93, 5.86, 5.51],
            'distance': 4.47
        },
    9245: {
            'time': [5.56, 5.83, 5.53],
            'distance': 7.86
        },
    10138: {
            'time': [6.96, 6.9, 6.74],
            'distance': 12.08
        },
    10994: {
            'time': [6.83, 6.61, 6.03],
            'distance': 5.17
        },
    11812: {
            'time': [6.78, 6.84, 7.05],
            'distance': 7.03
        },
    12644: {
            'time': [6.9, 7.04, 7.17],
            'distance': 2.61
        },
    12956: {
            'time': [7.78, 8.08, 8.01],
            'distance': 8.1
        },
    # 13464: {
    #         'time': [9.64, 9.63, 9.45],
    #         'distance': 69.92
    #     },
    14063: {
            'time': [7.67, 10.3, 8.29],
            'distance': 17.82
        },
    14394: {
            'time': [13, 8.77, 9.73],
            'distance': 3.27
        }
    
}

path_data = {}
for k in sorted(path_data_raw.keys()):
    print k
    v = path_data_raw[k]
    v_mean = np.mean(v['time'])
    v_std = np.std(v['time'])

    path_data[k] = {
        'time_mean': v_mean,
        'time_std': v_std,
        'distance': v['distance']
    }

print path_data

cloud_sizes = sorted(path_data.keys())
v_mean = []
v_std = []
v_distance = []

for k in cloud_sizes:
    v = path_data[k]
    v_mean.append(v['time_mean'])
    v_std.append(v['time_std'])
    v_distance.append(v['distance'])

print v_mean

# fig = plt.figure(figsize=(6, 6))
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter(cloud_sizes, v_distance, v_mean,
#            linewidths=1, alpha=.7,
#            edgecolor='k',
#            s = 200,
#            c=v_mean)

# ax.set_xlabel('Graph size (number of nodes)', fontsize=12)
# ax.set_ylabel('Path size (meters)', fontsize=12)
# ax.set_zlabel('Mean time (seconds)', fontsize=12)

# plt.show()


fig, ax = plt.subplots(1)
ax.grid()
ax.set_axisbelow(True)
scatter = plt.scatter(cloud_sizes, v_mean,
           linewidths=1, alpha=.7,
           edgecolor='k',
           s = 130,
           c=v_distance)

cbar = plt.colorbar(scatter)
cbar.set_label('Path size (meters)')
ax.set_ylabel('Time (seconds)', fontsize=12)
ax.set_xlabel('Graph Size (nodes)', fontsize=12)
plt.xticks(rotation=45)
ax.set_title('Planning time over the mesh graph', fontweight='bold', size=14)
fig.subplots_adjust(bottom=0.2)


plt.show()


# fig1, ax1 = plt.subplots(1)

# coef = np.polyfit(cloud_sizes, v_mean, 1)
# poly1d_fn = np.poly1d(coef) 

# #ax1.plot(cloud_sizes, v_mean, "-", cloud_sizes, poly1d_fn(cloud_sizes), '--k')
# ax1.plot(cloud_sizes, v_mean, "-")
# ax1.plot(cloud_sizes, v_mean, "o", markersize=4)

# array_v_mean = np.array(v_mean)
# array_v_std = np.array(v_std)
# ax1.fill_between(cloud_sizes, array_v_mean - array_v_std, array_v_mean + array_v_std, alpha=0.3)

# ax1.set_xlim([0, max(cloud_sizes) + 4000])
# ax1.set_ylim([0, max(v_mean) + max(v_mean) * 0.1])

# ax1.grid()
# ax1.set_axisbelow(True)
# ax1.set_ylabel('Time (seconds)', fontsize=12)
# ax1.set_xlabel('Cloud Size (points)', fontsize=12)
# plt.xticks(rotation=45)
# ax1.set_title('Reconstruction time (Indoor Multi-level scenario)', fontweight='bold', size=14)
# #ax1.legend(loc='upper right', fontsize=12)
# fig1.show()

fig1.subplots_adjust(bottom=0.2)
plt.show()
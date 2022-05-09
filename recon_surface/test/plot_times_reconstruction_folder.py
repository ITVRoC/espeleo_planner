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


cloud_reconstruction_data_raw = {
    120587: [49.450631, 51.448593, 51.241609],
    154520: [61.254180, 60.936983, 60.522624],
    135500: [42.527933, 43.042461, 42.824107],
    106674: [33.141320, 33.241263, 33.187268],
    66364: [18.096845, 17.766575, 17.949623],
    75930: [20.698809, 20.797669, 20.462283],
    54340: [20.141502, 19.399728, 19.395195],
    89575: [40.112978, 41.780798, 40.615911],
    38350: [11.885967, 11.858403, 11.829116],
    143193: [47.895612, 46.100656, 46.406983],
    96801: [36.662099, 35.796833, 36.497610],
    83254: [27.863079, 25.256210, 24.795236],
    204742: [89.870718, 102.393646, 97.171390],
    3631: [2.655282, 2.792699, 2.655125],
}

cloud_reconstruction_data = {}
for k in sorted(cloud_reconstruction_data_raw.keys()):
    print k
    v = cloud_reconstruction_data_raw[k]
    v_mean = np.mean(v)
    v_std = np.std(v)

    cloud_reconstruction_data[k] = {
        'mean': v_mean,
        'std': v_std
    }

print cloud_reconstruction_data

cloud_sizes = sorted(cloud_reconstruction_data.keys())
v_mean = []
v_std = []

for k in cloud_sizes:
    v = cloud_reconstruction_data[k]
    v_mean.append(v['mean'])
    v_std.append(v['std'])

print v_mean


fig1, ax1 = plt.subplots(1)

coef = np.polyfit(cloud_sizes, v_mean, 1)
poly1d_fn = np.poly1d(coef) 

#ax1.plot(cloud_sizes, v_mean, "-", cloud_sizes, poly1d_fn(cloud_sizes), '--k')
ax1.plot(cloud_sizes, v_mean, "-")
ax1.plot(cloud_sizes, v_mean, "o", markersize=4)

array_v_mean = np.array(v_mean)
array_v_std = np.array(v_std)
ax1.fill_between(cloud_sizes, array_v_mean - array_v_std, array_v_mean + array_v_std, alpha=0.3)

ax1.set_xlim([0, max(cloud_sizes) + 4000])
ax1.set_ylim([0, max(v_mean) + max(v_mean) * 0.1])

ax1.grid()
ax1.set_axisbelow(True)
ax1.set_ylabel('Time (seconds)', fontsize=12)
ax1.set_xlabel('Cloud Size (points)', fontsize=12)
plt.xticks(rotation=45)
ax1.set_title('Reconstruction time (Indoor Multi-level scenario)', fontweight='bold', size=14)
#ax1.legend(loc='upper right', fontsize=12)
fig1.show()

# ax2.grid()
# ax2.set_axisbelow(True)
# ax2.set_ylabel('Odometry distance (meters)')
# ax2.set_xlabel('Timesteps')
# ax2.set_title('Exploration odometry distance', fontweight='bold')
# ax2.legend(loc='upper left', fontsize=8)
# fig2.show()
#
# ax3.grid()
# ax3.set_axisbelow(True)
# ax3.set_ylabel('Actions taken')
# ax3.set_xlabel('Timesteps')
# ax3.set_title('Exploration actions', fontweight='bold')
# ax3.legend(loc='upper left', fontsize=8)
# fig3.show()

fig1.subplots_adjust(bottom=0.2)
plt.show()
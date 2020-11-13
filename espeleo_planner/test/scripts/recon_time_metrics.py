#!/usr/bin/env python

import rospy
import traceback
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import pcl
import pcl.pcl_visualization
import time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import matplotlib.pyplot as plt
import tf
from geometry_msgs.msg import Twist
import os
import json
import sys
import math
import argparse
import scipy.stats


if __name__ == '__main__':
    cave_1 = [28.625910, 29.610291, 30.753096, 29.655921, 28.881140, 29.920967, 29.508694, 33.534120, 30.807954, 30.134508]
    cave_1_points = 122084
    print "cave_1 points",  cave_1_points, "mean:", np.mean(np.asarray(cave_1)), np.std(np.asarray(cave_1))

    cave_2 = [38.901599, 38.919334, 36.843395, 37.258926, 40.562865, 37.808137, 37.526164, 38.104334, 37.427828, 37.194297]
    cave_2_points = 142124
    print "cave_2 points", cave_2_points, "mean:", np.mean(np.asarray(cave_2)), np.std(np.asarray(cave_2))

    cave_3 = [13.674770, 11.400837, 11.853093, 11.264136, 11.563486, 11.811291, 11.412711, 11.278657, 11.307714, 10.917437]
    cave_3_points = 42000
    print "cave_3 points", cave_3_points, "mean:", np.mean(np.asarray(cave_3)), np.std(np.asarray(cave_3))
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


def dir_path(path):
    if os.path.isdir(path):
        return path.rstrip('/')
    else:
        raise argparse.ArgumentTypeError("readable_dir:{} is not a valid path".format(path))


def mean_confidence_interval(data, confidence=0.95):
    return scipy.stats.t.interval(confidence, len(data) - 1, loc=np.mean(data), scale=scipy.stats.sem(data))


def process_experiment_folder(path, base_name):
    accumulated_data = {}

    print "base_name:", base_name

    file_count = 0
    for file in os.listdir(path):
        if file.endswith(".json"):
            file_count += 1
            json_path = os.path.join(path, file)
            print "\t", file_count, json_path

            with open(json_path) as json_file:
                data = json.load(json_file)

            sorted_data = sorted(data, key=lambda k: k['timestep'])

            for e in sorted_data:
                timestep = e['timestep']
                rmse = e['rmse']
                odom_dist = e['odom_dist']
                robot_action_number = e['robot_action_number']

                if timestep not in accumulated_data.keys():
                    accumulated_data[timestep] = {
                        'rmse': [rmse],
                        'odom_dist': [odom_dist],
                        'robot_action_number': [robot_action_number]
                    }
                else:
                    accumulated_data[timestep]['rmse'].append(rmse)
                    prev_odom = 0
                    if timestep > 0:
                        prev_odom = accumulated_data[timestep - 1]['odom_dist'][-1]
                    accumulated_data[timestep]['odom_dist'].append(odom_dist + prev_odom)
                    accumulated_data[timestep]['robot_action_number'].append(robot_action_number)

    mean_data = {}
    for timestep in sorted(accumulated_data.keys()):
        rmse = np.array(accumulated_data[timestep]['rmse'])
        rmse_mean = np.mean(rmse)
        rmse_std = np.std(rmse)
        rmse_ci_down, rmse_ci_up = mean_confidence_interval(rmse)

        odom_dist = np.array(accumulated_data[timestep]['odom_dist'])
        odom_dist_mean = np.mean(odom_dist)
        odom_dist_std = np.std(odom_dist)
        odom_ci_down, odom_ci_up = mean_confidence_interval(odom_dist)

        robot_action_number = np.array(accumulated_data[timestep]['robot_action_number'])
        robot_action_number_mean = np.mean(robot_action_number)
        robot_action_number_std = np.std(robot_action_number)
        robot_action_number_ci_down, robot_action_number_ci_up = mean_confidence_interval(robot_action_number)

        mean_data[timestep] = {
            'rmse_mean': rmse_mean,
            'rmse_std': rmse_std,
            'rmse_ci_down': rmse_ci_down,
            'rmse_ci_up': rmse_ci_up,

            'odom_dist_mean': odom_dist_mean,
            'odom_dist_std': odom_dist_std,
            'odom_ci_down': odom_ci_down,
            'odom_ci_up': odom_ci_up,

            'robot_action_number_mean': robot_action_number_mean,
            'robot_action_number_std': robot_action_number_std,
            'robot_action_number_ci_down': robot_action_number_ci_down,
            'robot_action_number_ci_up': robot_action_number_ci_up
        }

    return mean_data


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument("-f", required=True, help="folders to process", type=dir_path, nargs='+')

    args, unknown = ap.parse_known_args()
    print "args:", args

    fig1, ax1 = plt.subplots(1)
    # fig2, ax2 = plt.subplots(1)
    # fig3, ax3 = plt.subplots(1)

    total_mean_data = {}
    timesteps = None

    for f_path in args.f:
        base_name = os.path.basename(f_path)
        data = process_experiment_folder(f_path, base_name)
        total_mean_data[base_name] = data

        timesteps = sorted(data.keys())

        rmse_mean = []
        rmse_std = []
        rmse_ci_down = []
        rmse_ci_up = []
        for t in timesteps:
            rmse_mean.append(data[t]['rmse_mean'])
            rmse_std.append(data[t]['rmse_std'])
            rmse_ci_down.append(data[t]['rmse_ci_down'])
            rmse_ci_up.append(data[t]['rmse_ci_up'])

        rmse_mean = np.array(rmse_mean)
        rmse_std = np.array(rmse_std)

        if base_name.endswith("combined"):
            base_name = "Combined metric only"

        if base_name.endswith("random"):
            base_name = "Random"

        if base_name.endswith("metric"):
            base_name = "Ours"

        if base_name.endswith("short"):
            base_name = "Greedy"

        ax1.plot(timesteps, rmse_mean, label=base_name)
        #ax1.fill_between(timesteps, rmse_mean - rmse_std, rmse_mean + rmse_std, alpha=0.3)
        #ax1.fill_between(timesteps, rmse_ci_down, rmse_ci_up, alpha=0.3)
        ax1.set_xlim([0, max(timesteps) + 0.1])
        ax1.set_ylim([0, max(rmse_mean) + max(rmse_mean) * 0.1])

        # odom_dist_mean = []
        # odom_dist_std = []
        # for t in timesteps:
        #     odom_dist_mean.append(data[t]['odom_dist_mean'])
        #     odom_dist_std.append(data[t]['odom_dist_std'])
        #
        # odom_dist_mean = np.array(odom_dist_mean)
        # odom_dist_std = np.array(odom_dist_std)
        #
        # ax2.plot(timesteps, odom_dist_mean, label=base_name)
        # ax2.fill_between(timesteps, odom_dist_mean - odom_dist_std, odom_dist_mean + odom_dist_std, alpha=0.3)
        # ax2.set_xlim([0, max(timesteps) + 0.1])
        # ax2.set_ylim([0, max(odom_dist_mean) + max(odom_dist_mean) * 0.25])
        #
        # robot_action_number_mean = []
        # robot_action_number_std = []
        # for t in timesteps:
        #     robot_action_number_mean.append(data[t]['robot_action_number_mean'])
        #     robot_action_number_std.append(data[t]['robot_action_number_std'])
        #
        # robot_action_number_mean = np.array(robot_action_number_mean)
        # robot_action_number_std = np.array(robot_action_number_std)
        #
        # ax3.plot(timesteps, robot_action_number_mean, label=base_name)
        # ax3.fill_between(timesteps, robot_action_number_mean - robot_action_number_std, robot_action_number_mean + robot_action_number_std, alpha=0.3)
        # ax3.set_xlim([0, max(timesteps) + 0.1])
        # ax3.set_ylim([0, max(robot_action_number_mean) + max(robot_action_number_mean) * 0.1])

    ax1.grid()
    ax1.set_axisbelow(True)
    ax1.set_ylabel('RMSE', fontsize=12)
    ax1.set_xlabel('Timesteps', fontsize=12)
    #ax1.set_title('Exploration coverage error (Single-level Cave)', fontweight='bold', size=14)
    ax1.set_title('Exploration coverage error (Synthetic Cave)', fontweight='bold', size=14)
    ax1.legend(loc='upper right', fontsize=12)

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

    plt.show()
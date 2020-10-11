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


class ExplorationMetrics:

    def __init__(self, ground_truth_pointcloud_path, json_folder):
        self.ground_truth_pointcloud_path = ground_truth_pointcloud_path
        self.ground_truth_pointcloud = None
        self.current_map_pointcloud_msg = None

        self.json_filename = os.path.basename(ground_truth_pointcloud_path) + "_" + \
                             time.strftime("%d.%m.%Y-%H.%M.%S") + ".json"
        self.json_filename = os.path.join(json_folder, self.json_filename)

        self.init_timestamp = None
        self.current_timestep = 0

        self.pointcloud_plot_data = []

        self.odom_msg = None
        self.last_odom_msg = None

        self.cmd_vel_msg = None

        self.diff_percent = 0
        self.odom_dist = 0

        self.init_node()

    def init_node(self):
        """Init node with ROS bindings
        :return:
        """
        rospy.Subscriber('/laser_cloud_surround2', PointCloud2, self.pointcloud2_callback)
        rospy.Subscriber('/integrated_to_init2', Odometry, self.odom_callback)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        self.load_gt_pointcloud()

    def odom_callback(self, msg):
        self.odom_msg = msg

    def pointcloud2_callback(self, msg):
        self.current_map_pointcloud_msg = msg

        if not self.init_timestamp:
            self.init_timestamp = time.time()

    def cmd_vel_callback(self, msg):
        self.cmd_vel_msg = msg

    def load_gt_pointcloud(self):
        self.ground_truth_pointcloud = pcl.load(self.ground_truth_pointcloud_path)

        gt_size = self.ground_truth_pointcloud.size
        rospy.loginfo("GT pointcloud size:%s", gt_size)

        if gt_size <= 0.0:
            raise Exception("gt_size <= 0.0")

    def process_data(self):
        if not self.current_map_pointcloud_msg:
            rospy.logerr("No current_map_pointcloud_msg received")
            return False

        if not self.cmd_vel_msg:
            rospy.logerr("No cmd_vel_msg received")
            return False

        if not self.last_odom_msg:
            self.last_odom_msg = self.odom_msg
            return False

        # else:
        #     last_point = (self.last_odom_msg.pose.pose.position.x, self.last_odom_msg.pose.pose.position.y)
        #     curr_point = (self.odom_msg.pose.pose.position.x, self.odom_msg.pose.pose.position.y)
        #     odom_diff_xy = abs(last_point[0] - curr_point[0]) + abs(last_point[1] - curr_point[1])
        #
        #     _, _, last_y = tf.transformations.euler_from_quaternion([self.last_odom_msg.pose.pose.orientation.x,
        #                                                              self.last_odom_msg.pose.pose.orientation.y,
        #                                                              self.last_odom_msg.pose.pose.orientation.z,
        #                                                              self.last_odom_msg.pose.pose.orientation.w])
        #
        #     _, _, curr_y = tf.transformations.euler_from_quaternion([self.odom_msg.pose.pose.orientation.x,
        #                                                              self.odom_msg.pose.pose.orientation.y,
        #                                                              self.odom_msg.pose.pose.orientation.z,
        #                                                              self.odom_msg.pose.pose.orientation.w])
        #
        #     odom_diff_yaw = abs(last_y - curr_y)
        #     if odom_diff_xy < 0.1 and odom_diff_yaw < 0.6:
        #         rospy.logwarn("No movement detected on XY (%s) and yaw (%s)", odom_diff_xy, odom_diff_yaw)
        #         rospy.logwarn("last_y (%s) curr_y (%s)", last_y, curr_y)
        #         return
        # self.last_odom_msg = self.odom_msg

        if abs(self.cmd_vel_msg.linear.x) < 0.05 and abs(self.cmd_vel_msg.angular.z) < 0.05:
            rospy.logwarn("No movement lin.x:%.2f ang.z:%.2f  diff:%.2f%% odom_dist:%.2f",
                          self.cmd_vel_msg.linear.x,
                          self.cmd_vel_msg.angular.z,
                          self.diff_percent,
                          self.odom_dist)
            return False

        last_point = (self.last_odom_msg.pose.pose.position.x,
                      self.last_odom_msg.pose.pose.position.y,
                      self.last_odom_msg.pose.pose.position.z)
        curr_point = (self.odom_msg.pose.pose.position.x,
                      self.odom_msg.pose.pose.position.y,
                      self.odom_msg.pose.pose.position.z)
        local_odom_dist = math.sqrt((last_point[0] - curr_point[0]) ** 2 +
                                    (last_point[1] - curr_point[1]) ** 2 +
                                    (last_point[2] - curr_point[2]) ** 2)
        self.odom_dist += local_odom_dist
        self.last_odom_msg = self.odom_msg

        points = pc2.read_points_list(self.current_map_pointcloud_msg, field_names=("x", "y", "z"), skip_nans=True)
        cloud = pcl.PointCloud(np.array(points, dtype=np.float32))

        curr_pt_size = cloud.size
        gt_size = self.ground_truth_pointcloud.size

        self.diff_percent = (curr_pt_size / float(gt_size)) * 100
        #timestep = time.time() - self.init_timestamp

        self.pointcloud_plot_data.append({'timestep': self.current_timestep,
                                          'diff_percent': self.diff_percent,
                                          'odom_dist': local_odom_dist})

        rospy.loginfo("timestep:%s diff:%.2f%% odom_dist:%.2f", self.current_timestep, self.diff_percent, self.odom_dist)

        self.current_timestep += 1
        return True

    def save_json_to_file(self):
        with open(self.json_filename, 'w') as outfile:
            json.dump(self.pointcloud_plot_data, outfile)

        rospy.loginfo("Saved json to:%s", self.json_filename)

        return self.json_filename

    @staticmethod
    def plot_data_from_json_file(json_filepath):

        with open(json_filepath) as json_file:
            data = json.load(json_file)

        x = []
        y = []
        odom_dist = []
        for e in data:
            x.append(e['timestep'])
            y.append(e['diff_percent'])
            odom_dist.append(e['odom_dist'])

        rospy.loginfo("total odom_dist:%s", sum(odom_dist))

        fig, ax = plt.subplots()

        ax.plot(x, y)
        plt.fill_between(x, y, color='green', alpha=0.5)

        ax.set_ylim([0.0, 101])
        plt.xlim([0, max(x) + 0.1])

        ax.grid()
        ax.set_axisbelow(True)
        ax.set_ylabel('Coverage %')
        ax.set_xlabel('Timestep')
        ax.set_title('Exploration coverage', fontweight='bold')
        #fig.tight_layout()
        plt.savefig(json_filepath + "_odom_{:.3f}_timesteps_{:.3f}_plot.pdf".format(sum(odom_dist), max(x)))
        plt.show()

    def get_diff_percent(self):
        return self.diff_percent


if __name__ == '__main__':
    rospy.init_node('espeleo_exploration_metrics', anonymous=True)
    rospy.loginfo("espeleo_exploration_metrics node start")

    gt_pointcloud_file = "simple_cave_delimited_v2.ply"
    gt_pointcloud_folder = "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/comparison_pointclouds/"
    save_json_metrics_folder = "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/comparison_pointclouds/json/map_delimited_v2_short"
    exp_metrics = ExplorationMetrics(os.path.join(gt_pointcloud_folder, gt_pointcloud_file), save_json_metrics_folder)

    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--input", nargs='?', required=False, help="input json file")

    args, unknown = ap.parse_known_args()
    print(args)

    if args.input:
        # if there is a parameter for this script
        rospy.loginfo("Only reading json file: %s", args.input)
        exp_metrics.plot_data_from_json_file(args.input)
        sys.exit(0)

    rate = rospy.Rate(1.0)

    while not rospy.is_shutdown():
        try:
            time1 = time.time()
            res = exp_metrics.process_data()
            time2 = time.time()
            if res:
                rospy.loginfo('process_data %0.3f ms' % ((time2 - time1) * 1000.0))

            if exp_metrics.get_diff_percent() >= 100.0:
                rospy.logwarn("diff percent >= 100: %s", exp_metrics.get_diff_percent())
                break

        except Exception as e:
            tb = traceback.format_exc()
            rospy.logerr("Main Exception: %s", str(tb))

        rate.sleep()

    fname = exp_metrics.save_json_to_file()
    exp_metrics.plot_data_from_json_file(fname)

    rospy.loginfo("espeleo_exploration_metrics node stop")

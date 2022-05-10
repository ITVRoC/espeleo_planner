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
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Twist
import os
import json
import sys
import math
import argparse
import datetime


class ExplorationMetrics:

    def __init__(self, ground_truth_pointcloud_path, json_folder, leaf_size=0.6):
        self.ground_truth_pointcloud_path = ground_truth_pointcloud_path
        self.ground_truth_pointcloud = None
        self.current_map_pointcloud_msg = None
        self.leaf_size = leaf_size

        self.json_filename = os.path.basename(ground_truth_pointcloud_path) + "_" + \
                             time.strftime("%d.%m.%Y-%H.%M.%S") + ".json"
        self.json_filename = os.path.join(json_folder, self.json_filename)

        self.init_timestamp = None
        self.current_timestep = 0

        self.frontiers_ground_centroids_msg = None
        self.robot_action_number = 0

        self.pointcloud_plot_data = []

        self.odom_msg = None
        self.last_odom_msg = None

        self.cmd_vel_msg = None
        self.last_time_cmd_vel_updated = time.time()

        self.rmse = float('inf')
        self.odom_dist = 0
        self.last_time_updated = time.time()
        self.start_time = time.time()

        self.init_node()

    def init_node(self):
        """Init node with ROS bindings
        :return:
        """
        rospy.Subscriber('/cloud_transformed_world', PointCloud2, self.pointcloud2_callback)
        rospy.Subscriber('/integrated_to_init2', Odometry, self.odom_callback)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        rospy.Subscriber('/frontiers_ground_centroids', MarkerArray, self.frontier_cetntroids_callback)
        self.load_gt_pointcloud()

    def odom_callback(self, msg):
        self.odom_msg = msg

    def pointcloud2_callback(self, msg):
        self.current_map_pointcloud_msg = msg

        if not self.init_timestamp:
            self.init_timestamp = time.time()

    def cmd_vel_callback(self, msg):
        self.cmd_vel_msg = msg
        self.last_time_cmd_vel_updated = time.time()

    def frontier_cetntroids_callback(self, msg):
        self.frontiers_ground_centroids_msg = msg
        self.robot_action_number += 1

    def load_gt_pointcloud(self):
        self.ground_truth_pointcloud = pcl.load(self.ground_truth_pointcloud_path)
        self.ground_truth_pointcloud = ExplorationMetrics.voxelize_cloud(self.ground_truth_pointcloud, self.leaf_size)

        gt_size = self.ground_truth_pointcloud.size
        rospy.loginfo("GT pointcloud size:%s", gt_size)

        if gt_size <= 0.0:
            raise Exception("gt_size <= 0.0")

    @staticmethod
    def voxelize_cloud(cloud, leaf_size):
        vg = cloud.make_voxel_grid_filter()
        vg.set_leaf_size(leaf_size, leaf_size, leaf_size)
        cloud = vg.filter()

        return cloud

    @staticmethod
    def sim_outlier(cloud_a, cloud_b):
        kd = cloud_a.make_kdtree_flann()
        current_sum = 0

        # find the single closest points to each point in point cloud 2
        # (and the sqr distances)
        indices, sqr_distances = kd.nearest_k_search_for_cloud(cloud_b, 1)
        for i in range(cloud_b.size):
            current_sum += sqr_distances[i, 0]

        return math.sqrt(current_sum / cloud_a.size)

    @staticmethod
    def similarity(cloud_a, cloud_b):
        # compare B to A
        similarityB2A = ExplorationMetrics.sim_outlier(cloud_a, cloud_b)
        print("similarityB2A:", similarityB2A)

        # compare A to B
        similarityA2B = ExplorationMetrics.sim_outlier(cloud_b, cloud_a)
        print("similarityA2B:", similarityA2B)

        return (similarityA2B * 0.5) + (similarityB2A * 0.5)

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
            rospy.logwarn("No movement (time:%.2f) x:%.2f z:%.2f odom_dist:%.2f rmse:%.2f",
                          self.current_timestep,
                          self.cmd_vel_msg.linear.x,
                          self.cmd_vel_msg.angular.z,
                          self.odom_dist,
                          self.rmse)
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
        cloud = ExplorationMetrics.voxelize_cloud(cloud, self.leaf_size)
        pcl.save(cloud, "/tmp/world_map.ply", "ply")

        self.rmse = ExplorationMetrics.similarity(self.ground_truth_pointcloud, cloud)

        self.pointcloud_plot_data.append({'timestep': self.current_timestep,
                                          'rmse': self.rmse,
                                          'odom_dist': local_odom_dist,
                                          'robot_action_number': self.robot_action_number,
                                          'real_time': time.time()})

        rospy.loginfo("timestep:%s action_num:%s odom_dist:%.2f rmse:%.2f", self.current_timestep,
                      self.robot_action_number, self.odom_dist, self.rmse)
        
        rospy.loginfo("Real time so far: %s", str(datetime.timedelta(seconds=time.time()-self.start_time)))

        self.current_timestep += 1
        self.last_time_updated = time.time()
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
        action_num = []
        real_time = []
        for e in data:
            x.append(e['timestep'])
            y.append(e['rmse'])
            odom_dist.append(e['odom_dist'])
            action_num.append(e['robot_action_number'])
            real_time.append(e['real_time'])

        rospy.loginfo("total odom_dist:%s", sum(odom_dist))

        fig, ax = plt.subplots()

        ax.plot(x, y)
        #plt.fill_between(x, y, color='green', alpha=0.5)

        ax.set_ylim([0.0, max(y) + 1])
        plt.xlim([0, max(x) + 0.1])

        print("max(x):", max(x))

        print("Real time in this experiment was:", str(datetime.timedelta(seconds=real_time[-1]-real_time[0])))

        ax.grid()
        ax.set_axisbelow(True)
        ax.set_ylabel('RMSE')
        ax.set_xlabel('Timestep')
        ax.set_title('Exploration coverage', fontweight='bold')
        #fig.tight_layout()
        plt.savefig(json_filepath +
                    "_odom_{:.3f}_timesteps_{:.3f}_action_num_{:.3f}_plot.pdf".format(
                        sum(odom_dist), max(x), max(action_num)))
        plt.show()

    def get_rmse(self):
        return self.rmse

    def get_current_timestep(self):
        return self.current_timestep

    def get_seconds_since_last_update(self):
        return time.time() - self.last_time_updated

    def finalize_data(self, max_timesteps):
        last_data = self.pointcloud_plot_data[-1]
        last_timestep = last_data['timestep']

        len_diff = max_timesteps - len(self.pointcloud_plot_data)
        if len_diff > 0:
            rospy.logwarn('Augmenting data %s timesteps', len_diff)
            for i in range(len_diff):
                d = {
                    'timestep': last_timestep + (i + 1),
                    'rmse': last_data['rmse'],
                    'odom_dist': last_data['odom_dist'],
                    'robot_action_number': last_data['robot_action_number'],
                    'real_time': time.time()
                }
                self.pointcloud_plot_data.append(d)


if __name__ == '__main__':
    rospy.init_node('espeleo_exploration_metrics', anonymous=True)
    rospy.loginfo("espeleo_exploration_metrics node start")

    # map cave 1
    # gt_pointcloud_file = "rrt_simple_cave_delimited_v0.ply"
    # gt_pointcloud_folder = "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/comparison_pointclouds/reference_maps/"
    # save_json_metrics_folder = "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/comparison_pointclouds/json/map_delimited_rrt_v0_metric/"

    # map stage
    # gt_pointcloud_file = "stage_cave_v0_8m.ply"
    # gt_pointcloud_folder = "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/comparison_pointclouds/reference_maps/"
    # save_json_metrics_folder = "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/comparison_pointclouds/json/stage_map_rrt_v0_metric/"

    # map inclined 2
    gt_pointcloud_file = "simple_cave_inclined_v3.ply"
    gt_pointcloud_folder = "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/comparison_pointclouds/reference_maps/"
    save_json_metrics_folder = "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/comparison_pointclouds/json/inclined_rrt_v0_metric/"
    
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

    max_timesteps = 2000
    max_stopped_time = 500

    while not rospy.is_shutdown():
        try:
            time1 = time.time()
            res = exp_metrics.process_data()
            time2 = time.time()
            if res:
                rospy.loginfo('process_data %0.3f ms' % ((time2 - time1) * 1000.0))

            # if exp_metrics.get_rmse() <= 0.25:
            #     rospy.logwarn("rmse <= 0.25: %s", exp_metrics.get_rmse())
            #     break

            if time.time() - exp_metrics.last_time_cmd_vel_updated > 10:
                rospy.logwarn("Setting cmd_vel as 0")
                exp_metrics.cmd_vel_msg = Twist()
                exp_metrics.last_time_cmd_vel_updated = time.time()

            is_ending_experiment = False

            if exp_metrics.get_current_timestep() >= max_timesteps:
                is_ending_experiment = True
                rospy.logwarn("ending experiment by timesteps %s", exp_metrics.get_current_timestep())

            if exp_metrics.get_seconds_since_last_update() > max_stopped_time:
                is_ending_experiment = True
                rospy.logwarn("ending experiment by max stopped time %s", exp_metrics.get_seconds_since_last_update())

            if is_ending_experiment:
                rospy.logwarn("ending experiment... timestep:%s seconds_since_update:%s",
                              exp_metrics.get_current_timestep(),
                              exp_metrics.get_seconds_since_last_update())
                break

        except Exception as e:
            tb = traceback.format_exc()
            rospy.logerr("Main Exception: %s", str(tb))

        rate.sleep()

    rospy.loginfo("Experiment duration:%s", time.time() - exp_metrics.start_time)

    exp_metrics.finalize_data(max_timesteps)
    fname = exp_metrics.save_json_to_file()
    exp_metrics.plot_data_from_json_file(fname)

    rospy.loginfo("espeleo_exploration_metrics node stop")

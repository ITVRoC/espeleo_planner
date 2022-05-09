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


def voxelize_cloud(cloud, leaf_size):
    vg = cloud.make_voxel_grid_filter()
    vg.set_leaf_size(leaf_size, leaf_size, leaf_size)
    cloud = vg.filter()

    return cloud


def get_clustered_cloud(cloud):

    tree = cloud.make_kdtree()
    ec = cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(1.5)
    ec.set_MinClusterSize(10)
    ec.set_MaxClusterSize(25000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    if len(cluster_indices) <= 0:
        print "\tlen(cluster_indices)", len(cluster_indices)
        return None

    max_cluster_idx = len(cluster_indices) - 1
    len_max_cluster = len(cluster_indices[max_cluster_idx])
    print 'cluster_indices :', len(cluster_indices), " count."
    print 'max_cluster_idx :', max_cluster_idx, " len:", len_max_cluster

    cluster_points = np.zeros((len_max_cluster, 3), dtype=np.float32)
    for j, idx in enumerate(cluster_indices[max_cluster_idx]):
        cluster_points[j][0] = cloud[idx][0]
        cluster_points[j][1] = cloud[idx][1]
        cluster_points[j][2] = cloud[idx][2]

    cloud_cluster = pcl.PointCloud()
    cloud_cluster.from_array(cluster_points)

    return cloud_cluster

# template<typename CloudT>
# float _similarity(const CloudT& cloudA, const CloudT& cloudB, float threshold)
# {
#   // compare B to A
#   int num_outlier = 0;
#   pcl::search::KdTree<typename CloudT::PointType> tree;
#   tree.setInputCloud(cloudA.makeShared());
#   auto sum = std::accumulate(cloudB.begin(), cloudB.end(), 0.0f, [&](auto current_sum, const auto& pt) {
#     const auto dist = nearestDistance(tree, pt);
#
#     if(dist < threshold)
#     {
#       return current_sum + dist;
#     }
#     else
#     {
#       num_outlier++;
#       return current_sum;
#     }
#   });
#
#   return sum / (cloudB.size() - num_outlier);
# }
#
# // comparing the clouds each way, A->B, B->A and taking the average
# template<typename CloudT>
# float similarity(const CloudT& cloudA, const CloudT& cloudB, float threshold = std::numeric_limits<float>::max())
# {
#   // compare B to A
#   const auto similarityB2A = _similarity(cloudA, cloudB, threshold);
#   // compare A to B
#   const auto similarityA2B = _similarity(cloudB, cloudA, threshold);
#
#   return (similarityA2B * 0.5f)

def sim_outlier(cloud_a, cloud_b):
    kd = cloud_a.make_kdtree_flann()
    current_sum = 0

    # find the single closest points to each point in point cloud 2
    # (and the sqr distances)
    indices, sqr_distances = kd.nearest_k_search_for_cloud(cloud_b, 1)
    for i in range(cloud_b.size):
        current_sum += sqr_distances[i, 0]

    return math.sqrt(current_sum / cloud_a.size)


def similarity(cloud_a, cloud_b):
    # compare B to A
    similarityB2A = sim_outlier(cloud_a, cloud_b)
    print "similarityB2A:", similarityB2A

    # compare A to B
    similarityA2B = sim_outlier(cloud_b, cloud_a)
    print "similarityA2B:", similarityA2B

    return (similarityA2B * 0.5) + (similarityB2A * 0.5)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument("-a", required=True, help="cloud a")
    ap.add_argument("-b", required=True, help="cloud b")

    leaf_size = 0.6

    args, unknown = ap.parse_known_args()
    print(args)

    c_a = pcl.load(args.a)
    c_b = pcl.load(args.b)

    time1 = time.time()
    c_a = voxelize_cloud(c_a, leaf_size)
    c_b = voxelize_cloud(c_b, leaf_size)
    res = similarity(c_a, c_b)
    time2 = time.time()
    print("diff:", res)
    print('process_data {:.3f} ms'.format((time2 - time1) * 1000.0))

    # time1 = time.time()
    # cloud_a = pcl.load(args.a)
    # print("cloud_a normal size:{}".format(cloud_a.size))
    # cloud_a = voxelize_cloud(cloud_a, leaf_size)
    # print("cloud_a voxelized size:{}".format(cloud_a.size))
    # # cloud_a = get_clustered_cloud(cloud_a)
    # # print("cloud_a normal size:{}".format(cloud_a.size))
    #
    # cloud_b = pcl.load(args.b)
    # print("cloud_b normal size:{}".format(cloud_b.size))
    # cloud_b = voxelize_cloud(cloud_b, leaf_size)
    # print("cloud_b voxelized size:{}".format(cloud_b.size))
    # # cloud_b = get_clustered_cloud(cloud_b)
    # # print("cloud_b normal size:{}".format(cloud_b.size))
    #
    # time2 = time.time()
    #
    # viewer = pcl.pcl_visualization.PCLVisualizering(b"3D Viewer")
    # viewer.InitCameraParameters()
    #
    # visual = pcl.pcl_visualization.CloudViewing()
    # visual.ShowMonochromeCloud(cloud_a, b'cloud_a')
    # # visual.ShowGrayCloud(ptcloud_centred, b'cloud')
    # visual.ShowMonochromeCloud(cloud_b, b'cloud_b')
    # # visual.ShowColorACloud(ptcloud_centred, b'cloud')
    #
    # v = True
    # while v:
    #     v = not (visual.WasStopped())
    #
    # print("diff %:",  (cloud_b.size / float(cloud_a.size)) * 100)
    # print('process_data {:.3f} ms'.format((time2 - time1) * 1000.0))

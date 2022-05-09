#!/usr/bin/env python

import sys
import os
import rospkg
import rospy
import pymesh
import networkx as nx
import numpy as np
from mayavi import mlab
from scipy import spatial
import pcl
import pcl.pcl_visualization
import graphviz
import pydot
import pygraphviz as pgv
import inspect
import open3d as o3d
import math
from sklearn.neighbors import NearestNeighbors

import matplotlib.pyplot as plt


rospack = rospkg.RosPack()

package_path = rospack.get_path('espeleo_planner')
sys.path.append(os.path.join(package_path, "test"))
sys.path.append(os.path.join(package_path, "scripts"))

import mesh_planner
from mesh_planner import graph_search

def create_graph(ply_path,
                 frontiers,
                 traversal_tresh=35):

    print("ply_path:", ply_path)
    print("frontiers:", frontiers)

    pcd = o3d.io.read_point_cloud(ply_path)
    print "pointcloud.has_normals:", bool(pcd.has_normals)

    downpcd = pcd.voxel_down_sample(voxel_size=0.25)
    print "downsampled normal:", downpcd.normals[0]

    filtered_points = []
    for i in range(len(downpcd.points)):
        normal = downpcd.normals[i]
        face_inclination = graph_search.MeshGraphSearch.calculate_traversal_angle(normal)
        if traversal_tresh < face_inclination < 180 - traversal_tresh:
            continue

        filtered_points.append(list(downpcd.points[i]))

    filteredpcd = o3d.geometry.PointCloud()
    filteredpcd.points = o3d.utility.Vector3dVector(np.asarray(filtered_points))

    frontier_locations = [
        mesh_planner.mesh_helper.find_closer_centroid(
            filtered_points, f_p, force_return_closer=True
        ) for f_p in frontiers
    ]
    p_clusters = [[] for _ in frontier_locations]

    for i, p in enumerate(filtered_points):

        min_idx = 0
        min_dist = float("inf")
        for j, f_p in enumerate(frontier_locations):
            a = np.asarray(filtered_points[i])

            frontier_idx = frontier_locations[j]
            b = np.asarray(filtered_points[frontier_idx])

            dist = np.linalg.norm(a - b)
            if dist < min_dist:
                min_dist = dist
                min_idx = j

        p_clusters[min_idx] += [filtered_points[i]]

    for c_idx, cluster_points in enumerate(p_clusters):
        print("c_idx:", c_idx, cluster_points)
        x, y, z = zip(*cluster_points)
        plt.scatter(x, y, s=0.5)

    plt.title("Cluster point cloud in {} frontiers".format(len(p_clusters)))
    plt.show()
    plt.close()


if __name__ == '__main__':
    rospy.init_node('plot_riemannian_node')
    rospy.loginfo("plot_riemannian_node start")

    test_files = [
        {
            "ply": "riemannian_pcloud.ply",
            "dot": "riemannian_graph.dot",
            "robot_pos": (-15, 5, 0),
            "frontiers": [(0.18, -4.2, -0.18), (3.35, 5.24, -0.03), (7.82, 2.54, -0.03)],
        }
    ]

    for test in test_files[:]:
        ply_path = os.path.join(package_path, "test", "point_cloud_planning", test["ply"])
        create_graph(ply_path, test["frontiers"])


#!/usr/bin/env python

import os
import rospkg
import sys
import traceback

import mesh_planner
import pymesh
import rospy
from geometry_msgs.msg import PoseStamped
from mesh_planner import mesh_helper, graph_metrics, mesh_planner_base, mesh_planner_node, pointcloud_planner_base
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

import sys
import os
import rospkg
import rospy
import pymesh
import networkx as nx
import numpy as np
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
from geometry_msgs.msg import Polygon, PointStamped

import mesh_planner

if __name__ == '__main__':
    rospy.init_node('test_point_cloud_planner_node')

    mplanner = mesh_planner_node.MeshPlannerNode()
    mplanner.init_manual_node()

    rate_slow = rospy.Rate(1.0)
    rate_fast = rospy.Rate(10.0)

    src = (-18.9, 6.1, -0.18)
    dst = (0.18, -4.2, -0.18)
    mplanner.clicked_point_msg = PointStamped()
    mplanner.clicked_point_msg.point.x = dst[0]
    mplanner.clicked_point_msg.point.y = dst[1]
    mplanner.clicked_point_msg.point.z = dst[2]

    test_pcloud_filename = "riemannian_pcloud.ply"

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('espeleo_planner')
    ply_path = os.path.join(pkg_path, "test", "point_cloud_planning", test_pcloud_filename)
    rospy.loginfo("ply_path: %s", ply_path)

    # load the mesh and locate the face closer to the src and dst points
    pcd = o3d.io.read_point_cloud(ply_path)
    downpcd = pcd.voxel_down_sample(voxel_size=0.30)
    filtered_points = downpcd.points

    np_points = np.asarray(downpcd.points)
    np_normals = np.asarray(downpcd.normals)

    print(("np_normals:", np_normals.shape))

    normalized_np_normals = (np_normals / np.linalg.norm(np_normals, axis=1)[:, None])
    clipped_u = np.clip(np.dot(normalized_np_normals, [0, 0, -1]), -1.0, 1.0)
    arccos_u = np.arccos(clipped_u)
    degrees_u = np.degrees(arccos_u)
    degrees_u = np.where(~(degrees_u > 90), degrees_u, np.abs(degrees_u - 180))

    np_points = np_points[(degrees_u <= 20)]
    np_normals = np_normals[(degrees_u <= 20)]

    filtered_downpcd = o3d.geometry.PointCloud()
    filtered_downpcd.points = o3d.utility.Vector3dVector(np_points)
    filtered_downpcd.normals = o3d.utility.Vector3dVector(np_normals)

    filtered_points = np.asarray(filtered_downpcd.points)

    while not rospy.is_shutdown():

        try:
            if not mplanner.clicked_point_msg:
                rospy.loginfo("Planner waiting for data...")
                rate_slow.sleep()
                continue

            rospy.loginfo("Start planning...")

            dst = (mplanner.clicked_point_msg.point.x,
                   mplanner.clicked_point_msg.point.y,
                   mplanner.clicked_point_msg.point.z)

            mplanner.clicked_point_msg = None

            src_marker = mesh_helper.create_marker((src[0], src[1], src[2] + 1.5),
                                                   color=(1.0, 0.0, 0.0),
                                                   duration=0,
                                                   marker_id=0,
                                                   m_scale=1)
            mplanner.pub_src_point.publish(src_marker)

            dst_marker = mesh_helper.create_marker((dst[0], dst[1], dst[2] + 1.5),
                                                   color=(0.654, 0.984, 0.137),
                                                   duration=0,
                                                   marker_id=1,
                                                   m_scale=1)
            mplanner.pub_dst_point.publish(dst_marker)

            rate_fast.sleep()

            source_face = mesh_helper.find_closer_centroid(filtered_points, src, force_return_closer=True)
            target_face = mesh_helper.find_closer_centroid(filtered_points, dst, force_return_closer=True)

            # check if src and dst faces are found
            if source_face == -1 or target_face == -1 or source_face == target_face:
                rospy.loginfo("Cannot find the target or source face: src:%d dst:%d", source_face, target_face)
                rate_slow.sleep()
                sys.exit()

            print(("source id:", source_face))
            print(("target id:", target_face))

            # graph_metric_types = [graph_metrics.GraphMetricType.SHORTEST,
            #                       graph_metrics.GraphMetricType.FLATTEST,
            #                       graph_metrics.GraphMetricType.ENERGY,
            #                       graph_metrics.GraphMetricType.COMBINED,
            #                       graph_metrics.GraphMetricType.STRAIGHTEST]

            #graph_metric_types = [graph_metrics.GraphMetricType.STRAIGHTEST]
            # graph_metric_types = [graph_metrics.GraphMetricType.SHORTEST]
            #graph_metric_types = [graph_metrics.GraphMetricType.COMBINED]

            # all flatest metrics
            # graph_metric_types = [graph_metrics.GraphMetricType.FLATTEST,
            #                       graph_metrics.GraphMetricType.FLATTEST_PYBULLET,
            #                       #graph_metrics.GraphMetricType.FLATTEST_PYBULLET_NORMAL,
            #                       graph_metrics.GraphMetricType.FLATTEST_OPTIMIZATION]
            #                       #graph_metrics.GraphMetricType.FLATTEST_OPTIMIZATION_NORMAL]

            #graph_metric_types = [graph_metrics.GraphMetricType.FLATTEST_PYBULLET_NORMAL,
            #                      graph_metrics.GraphMetricType.FLATTEST_OPTIMIZATION_NORMAL]

            #graph_metric_types = [graph_metrics.GraphMetricType.FLATTEST_PYBULLET_NORMAL]

            graph_metric_types = [graph_metrics.GraphMetricType.COMBINED]

            #sprint("prev downpcd:", type(filtered_downpcd))
            planner = pointcloud_planner_base.PointCloudPlannerBase(filtered_downpcd, graph_metric_types)

            target_frontiers = [target_face]
            target_weights = [1]
            return_dict = planner.run(source_face, target_frontiers, target_weights, is_debug=True)

            mplanner.publish_paths(return_dict)

            # rospy.signal_shutdown(0)
            # sys.exit()
        except Exception as e:
            tb = traceback.format_exc()
            rospy.logerr("Main Exception: %s", str(tb))

    rospy.loginfo("MeshPlanner node stop")

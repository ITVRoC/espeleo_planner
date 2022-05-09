#!/usr/bin/env python

import time
import rospy
import traceback
import pymesh
from recon_surface.srv import MeshFromPointCloud2
from visualization_msgs.msg import MarkerArray, Marker
import random
import numpy as np

from mesh_planner.graph_metrics import GraphMetricType
import espeleo_control.msg
import std_msgs.msg
from geometry_msgs.msg import Polygon, Point32, Point
from mesh_planner import mesh_helper, graph_metrics, mesh_planner_base, mesh_planner_node
from espeleo_planner.srv import processAllFrontiers, processAllFrontiersUsingSTLOctomap
from geometry_msgs.msg import Twist

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

from pyquaternion import Quaternion

map_pointcloud_msg = None
odom_msg = None
frontier_pts_arr = MarkerArray()
frontier_centroids_arr = MarkerArray()
frontier_centroids_arr_labels = MarkerArray()


def map_point_cloud_callback(msg):
    global map_pointcloud_msg
    map_pointcloud_msg = msg


def odom_callback(msg):
    global odom_msg
    odom_msg = msg


def rotation_matrix_from_vectors(vec1, vec2):
    """ Find the rotation matrix that aligns vec1 to vec2
    :param vec1: A 3d "source" vector
    :param vec2: A 3d "destination" vector
    :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
    """
    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    return rotation_matrix


def delete_previous_markers(marker_arr, publisher):
    if len(marker_arr.markers) > 0:
        for marker_idx in range(len(marker_arr.markers)):
            marker_arr.markers[marker_idx].action = Marker.DELETEALL

        publisher.publish(marker_arr)
        time.sleep(0.05)

    return MarkerArray()


if __name__ == '__main__':
    rospy.init_node('publish_frontiers_node')

    mplanner = mesh_planner_node.MeshPlannerNode()

    rospy.loginfo("publish_frontiers_node start")

    rate_very_slow = rospy.Rate(0.05)
    rate_slow = rospy.Rate(1.0)
    rate_fast = rospy.Rate(10.0)

    # subscribers
    rospy.Subscriber('/laser_cloud_surround2', PointCloud2, map_point_cloud_callback)
    #rospy.Subscriber('/full_cloud_projected2', PointCloud2, map_point_cloud_callback)
    #rospy.Subscriber('/octomap_point_cloud_centers', PointCloud2, map_point_cloud_callback)
    rospy.Subscriber('/integrated_to_init2', Odometry, odom_callback)

    rospy.sleep(3.)

    while not rospy.is_shutdown():
        try:
            # check if the source and target are valid
            if not map_pointcloud_msg or not odom_msg:
                rospy.loginfo("Mesh frontier estimation waiting for data...")
                rate_slow.sleep()
                continue

            curr_map = map_pointcloud_msg
            src = odom_msg

            map_pointcloud_msg = None
            odom_msg = None

            src_marker = mesh_helper.create_marker((src.pose.pose.position.x,
                                                    src.pose.pose.position.y,
                                                    src.pose.pose.position.z),
                                                   color=(0.0, 1.0, 0.0), duration=60, marker_id=0)
            mplanner.pub_src_point.publish(src_marker)
            rate_fast.sleep()  # allow publishing the markers without waiting to the last rate.sleep()

            # run the service to convert the point cloud to mesh
            mesh_filepath = None
            try:
                rospy.wait_for_service('/mesh_from_pointclouds', timeout=3)
                mesh_from_pointcloud = rospy.ServiceProxy('/mesh_from_pointclouds', MeshFromPointCloud2)
                mesh_src_point = Point(0.0, 0.0, 0.0)
                resp1 = mesh_from_pointcloud(curr_map, mesh_src_point)
                mesh_filepath = resp1.path
                rospy.loginfo("pointcloud processed result: %s", resp1)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s", e)
            except Exception as e:
                rospy.logerr("Exception: %s", e)

            if mesh_filepath is None:
                rospy.logerr("mesh_filepath is None, cannot continue with the planning")
                rate_slow.sleep()
                continue

            # load the mesh and locate the face closer to the src and dst points
            mesh_load = pymesh.load_mesh(mesh_filepath)
            mesh_load.add_attribute("face_centroid")
            mesh_load.add_attribute("face_normal")
            centroids = mesh_load.get_face_attribute("face_centroid")
            normals = mesh_load.get_face_attribute("face_normal")

            vertices = mesh_load.vertices
            ver_face = mesh_load.faces

            source_face = mesh_helper.find_closer_centroid(centroids,
                                                           (src.pose.pose.position.x,
                                                            src.pose.pose.position.y,
                                                            src.pose.pose.position.z),
                                                           force_return_closer=True)

            # check if src and dst faces are found
            if source_face == -1:
                rospy.loginfo("Cannot find the source face: src:%d", source_face)
                rate_slow.sleep()
                continue

            print("create_graph_from_mesh...")
            mplanner_base = mesh_planner_base.MeshPlannerBase(mesh_filepath, [graph_metrics.GraphMetricType.SHORTEST])
            G = mplanner_base.create_graph_from_mesh()
            print("prepare_graph...")
            G, f_visit_ids, f_centroids_ids, filtered_reachable_frontiers_ids = mplanner_base.prepare_graph(G, source_face)

            # mplanner_base.plot_graph_3d(G,
            #                       title="Frontier test",
            #                       source_id=source_face,
            #                       reachable_frontiers_ids=list(filtered_reachable_frontiers_ids),
            #                       frontier_centroids_ids=f_centroids_ids)

            rospy.loginfo("Estimated frontiers: %f", len(f_visit_ids))

            # delete previous frontier points
            frontier_pts_arr = delete_previous_markers(frontier_pts_arr, mplanner.pub_frontiers_ground_pts)

            # publish frontiers points
            for f_id in filtered_reachable_frontiers_ids:
                x, y, z = centroids[f_id]
                f_marker = mesh_helper.create_marker((x,
                                                      y,
                                                      z),
                                                     color=(0.7, 0.0, 0.0), duration=60, m_scale=0.5, marker_id=f_id)
                frontier_pts_arr.markers.append(f_marker)
            mplanner.pub_frontiers_ground_pts.publish(frontier_pts_arr)

            # delete previous centroid points
            frontier_centroids_arr = delete_previous_markers(frontier_centroids_arr,
                                                             mplanner.pub_frontiers_ground_centroids)
            # delete previous centroid labels
            frontier_centroids_arr_labels = delete_previous_markers(frontier_centroids_arr_labels,
                                                                    mplanner.pub_frontiers_ground_centroids_labels)
            # publish frontier centroids
            for i, f_id in enumerate(f_centroids_ids):
                x, y, z = centroids[f_id]
                R = rotation_matrix_from_vectors((0, 0, 1), normals[f_id])
                q = Quaternion(matrix=R)

                f_marker = mesh_helper.create_marker((x,
                                                      y,
                                                      z),
                                                     # orientation=[q[3], q[0], q[1], q[2]],
                                                     color=(0.8, 0.0, 0.0), duration=60, m_scale=0.8, marker_id=f_id,
                                                     marker_type=1)
                frontier_centroids_arr.markers.append(f_marker)

                f_label_marker = mesh_helper.create_marker((x,
                                                            y,
                                                            z + 4.0),
                                                           color=(0.7, 0.0, 0.0), duration=60, m_scale=0.8, marker_id=i,
                                                           marker_type=9, marker_text="Frontier {}".format(i))
                frontier_centroids_arr_labels.markers.append(f_label_marker)

            mplanner.pub_frontiers_ground_centroids.publish(frontier_centroids_arr)
            mplanner.pub_frontiers_ground_centroids_labels.publish(frontier_centroids_arr_labels)

            # run the service to convert the point cloud to mesh
            frontiers_mi = []
            # NORMAL OCTOMAP
            # try:
            #     rospy.wait_for_service('/process_all_frontiers', timeout=3)
            #     frontier_mi_service = rospy.ServiceProxy('/process_all_frontiers', processAllFrontiers)
            #     resp1 = frontier_mi_service()
            #     frontiers_mi = resp1.mi_data
            #     rospy.loginfo("process_all_frontiers result: %s %s", resp1, frontiers_mi)
            # except rospy.ServiceException as e:
            #     rospy.logerr("Service call failed: %s", e)
            # except Exception as e:
            #     rospy.logerr("Exception: %s", e)

            # STL OCTOMAP
            try:
                rospy.wait_for_service('/process_all_frontiers_stl', timeout=3)
                frontier_mi_service = rospy.ServiceProxy('/process_all_frontiers_stl',
                                                         processAllFrontiersUsingSTLOctomap)
                resp1 = frontier_mi_service(mesh_filepath)
                frontiers_mi = resp1.mi_data
                rospy.loginfo("process_all_frontiers result: %s %s", resp1, frontiers_mi)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s", e)
            except Exception as e:
                rospy.logerr("Exception: %s", e)

            if frontiers_mi is None or len(frontiers_mi) <= 0:
                rospy.logerr("process_all_frontiers is None, cannot continue with the planning")
                rate_slow.sleep()
                continue

            rospy.loginfo("Publish frontiers slow sleep...")
            rate_slow.sleep()
        except Exception as e:
            tb = traceback.format_exc()
            rospy.logerr("Main Exception: %s", str(tb))

        rate_slow.sleep()

    rospy.loginfo("MeshPlanner node stop")

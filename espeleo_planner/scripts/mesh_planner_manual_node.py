#!/usr/bin/env python

import rospy
import traceback
import pymesh
from recon_surface.srv import MeshFromPointCloud2
from mesh_planner import mesh_helper, graph_metrics, mesh_planner_base, mesh_planner_node

from mesh_planner.graph_metrics import GraphMetricType
import espeleo_control.msg
import std_msgs.msg
from geometry_msgs.msg import Polygon, Point32, Twist, Point

import pcl
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import ctypes
import struct
from sensor_msgs.msg import PointCloud2, PointField
from scipy.ndimage import gaussian_filter1d
import matplotlib.pyplot as plt


def smooth_curve(path_points, sigma=5):
    """
    Smooth a curve
    :param path_points:
    :return:
    """

    path_array = []
    p_z = []
    for p in path_points:
        path_array.append([p.x, p.y])
        p_z.append(p.z)
    path_array = np.array(path_array)

    x, y = path_array.T
    t = np.linspace(0, 1, len(x))
    t2 = np.linspace(0, 1, 100)

    x2 = np.interp(t2, t, x)
    y2 = np.interp(t2, t, y)

    x3 = gaussian_filter1d(x2, sigma)
    y3 = gaussian_filter1d(y2, sigma)

    x4 = np.interp(t, t2, x3)
    y4 = np.interp(t, t2, y3)

    plt.plot(x, y, "o-", lw=2)
    plt.plot(x3, y3, "r-", lw=2)
    plt.plot(x4, y4, "o", lw=2)
    plt.show()

    xyz = list(zip(x4, y4, p_z))
    final_path = []
    for e in xyz:
        final_path.append(Point32(e[0], e[1], e[2]))

    return final_path


def pcl_to_ros(pcl_array, frame_id="world"):
    """ Converts a ROS PointCloud2 message to a pcl PointXYZRGB

        Args:
            pcl_array (PointCloud_PointXYZRGB): A PCL XYZRGB point cloud

        Returns:
            PointCloud2: A ROS point cloud
    """
    ros_msg = PointCloud2()

    ros_msg.header.stamp = rospy.Time.now()
    ros_msg.header.frame_id = frame_id

    ros_msg.height = 1
    ros_msg.width = pcl_array.size

    ros_msg.fields.append(PointField(
        name="x",
        offset=0,
        datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
        name="y",
        offset=4,
        datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
        name="z",
        offset=8,
        datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
        name="rgb",
        offset=16,
        datatype=PointField.FLOAT32, count=1))

    ros_msg.is_bigendian = False
    ros_msg.point_step = 32
    ros_msg.row_step = ros_msg.point_step * ros_msg.width * ros_msg.height
    ros_msg.is_dense = False
    buffer = []

    for data in pcl_array:
        buffer.append(struct.pack('ffffBBBBIII', data[0], data[1], data[2], 1.0, 0, 0, 0, 0, 0, 0, 0))

    ros_msg.data = "".join(buffer)

    return ros_msg


def clip_pointcloud_msg(msg, src, clip_distance=8):
    """
    Clip pointcloud from source position given an offset
    This reduces the size of the pointcloud to a reazonable size to
    perform the reconstruction
    :param msg:
    :param x:
    :param y:
    :param z:
    :param offset:
    :return:
    """
    points = pc2.read_points_list(msg, field_names=("x", "y", "z"), skip_nans=True)
    cloud = pcl.PointCloud(np.array(points, dtype=np.float32))

    passthrough = cloud.make_passthrough_filter()
    passthrough.set_filter_field_name('x')
    passthrough.set_filter_limits(-clip_distance + src.pose.pose.position.x, clip_distance + src.pose.pose.position.x)
    cloud_filtered = passthrough.filter()

    passthrough = cloud_filtered.make_passthrough_filter()
    passthrough.set_filter_field_name('y')
    passthrough.set_filter_limits(-clip_distance + src.pose.pose.position.y, clip_distance + src.pose.pose.position.y)
    cloud_filtered = passthrough.filter()

    passthrough = cloud_filtered.make_passthrough_filter()
    passthrough.set_filter_field_name('z')
    #passthrough.set_filter_limits(-clip_distance + src.pose.pose.position.z, clip_distance + src.pose.pose.position.z)
    passthrough.set_filter_limits(-1.5 + src.pose.pose.position.z, 0.8 + src.pose.pose.position.z)
    cloud_filtered = passthrough.filter()

    vg = cloud_filtered.make_voxel_grid_filter()
    vg.set_leaf_size(0.05, 0.05, 0.05)
    cloud_filtered = vg.filter()

    filtered_msg = pcl_to_ros(cloud_filtered, frame_id=msg.header.frame_id)
    return filtered_msg


if __name__ == '__main__':
    rospy.init_node('mesh_planner_manual_main_node')

    mplanner = mesh_planner_node.MeshPlannerNode()
    mplanner.init_manual_node()
    mplanner.init_action_node()

    rospy.loginfo("MeshPlanner node start")

    rate_very_slow = rospy.Rate(0.05)
    rate_slow = rospy.Rate(1.0)
    rate_fast = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            # check if the source and target are valid
            if not mplanner.is_ready_to_plan():
                rospy.loginfo("Planner waiting for data...")
                rate_slow.sleep()
                continue

            rospy.loginfo("Start planning...")

            src, dst, pcloud = mplanner.get_plan_data()
            mplanner.reset_data()

            src_marker = mesh_helper.create_marker((src.pose.pose.position.x,
                                                    src.pose.pose.position.y,
                                                    src.pose.pose.position.z),
                                                   color=(0.0, 1.0, 0.0), duration=30, marker_id=0)
            mplanner.pub_src_point.publish(src_marker)

            dst_marker = mesh_helper.create_marker((dst.point.x,
                                                    dst.point.y,
                                                    dst.point.z),
                                                   color=(0.0, 0.0, 1.0), duration=30, marker_id=1)
            mplanner.pub_dst_point.publish(dst_marker)
            rate_fast.sleep()  # allow publishing the markers without waiting to the last rate.sleep()

            # run the service to convert the point cloud to mesh
            mesh_filepath = None
            try:
                filtered_pcloud = pcloud
                #filtered_pcloud = clip_pointcloud_msg(pcloud, src, clip_distance=12)
                rospy.wait_for_service('/mesh_from_pointclouds', timeout=3)
                mesh_from_pointcloud = rospy.ServiceProxy('/mesh_from_pointclouds', MeshFromPointCloud2)

                mesh_src_point = Point(src.pose.pose.position.x,
                                       src.pose.pose.position.y,
                                       src.pose.pose.position.z)
                resp1 = mesh_from_pointcloud(filtered_pcloud, mesh_src_point)
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
            centroids = mesh_load.get_face_attribute("face_centroid")

            vertices = mesh_load.vertices
            ver_face = mesh_load.faces

            source_face = mesh_helper.find_closer_centroid(centroids,
                                                          (src.pose.pose.position.x,
                                                           src.pose.pose.position.y,
                                                           src.pose.pose.position.z),
                                                          force_return_closer=True)

            target_face = mesh_helper.find_closer_centroid(centroids,
                                                          (dst.point.x,
                                                           dst.point.y,
                                                           dst.point.z),
                                                          force_return_closer=True)

            # check if src and dst faces are found
            if source_face == -1 or target_face == -1:
                rospy.loginfo("Cannot find the target or source face: src:%d dst:%d", source_face, target_face)
                rate_slow.sleep()
                continue

            # graph_metric_types = [graph_metrics.GraphMetricType.SHORTEST,
            #                  graph_metrics.GraphMetricType.FLATTEST,
            #                  graph_metrics.GraphMetricType.ENERGY,
            #                  graph_metrics.GraphMetricType.COMBINED,
            #                  graph_metrics.GraphMetricType.STRAIGHTEST]

            # graph_metric_types = [graph_metrics.GraphMetricType.STRAIGHTEST]
            #graph_metric_types = [graph_metrics.GraphMetricType.SHORTEST]
            graph_metric_types = [graph_metrics.GraphMetricType.COMBINED]

            planner = mesh_planner_base.MeshPlannerBase(mesh_filepath, graph_metric_types)

            # print "create_graph_from_mesh..."
            # G = planner.create_graph_from_mesh()
            # print "planner prepare_graph..."
            # G, f_visit_ids, f_centroids_ids, filtered_reachable_frontiers_ids = planner.prepare_graph(G, source_face)
            #
            # planner.plot_graph_3d(G,
            #                       title="Frontier test",
            #                       source_id=source_face,
            #                       reachable_frontiers_ids=list(filtered_reachable_frontiers_ids),
            #                       frontier_centroids_ids=f_centroids_ids)

            # repeat the paths 3 times for estimating metrics
            # for i in xrange(3):
            #     return_dict = planner.run(source_face, target_face, is_debug=False)
            #     mplanner.publish_paths(return_dict)
            #     #print "return_dict:", return_dict

            return_dict = planner.run(source_face, target_face, is_debug=False)
            mplanner.publish_paths(return_dict)

            # SEND PATH TO CONTROLLER # TESTING HECTOR

            path_msg = return_dict[graph_metrics.GraphMetricType.COMBINED]['path_msg']

            espeleo_path = espeleo_control.msg.Path()
            espeleo_path.header = std_msgs.msg.Header()
            espeleo_path.header.stamp = rospy.Time.now()
            espeleo_path.header.frame_id = path_msg.header.frame_id
            espeleo_path.closed_path_flag = False
            espeleo_path.insert_n_points = 10
            espeleo_path.filter_path_n_average = 5

            path_points = []
            for e in path_msg.poses:
                path_points.append(Point32(e.pose.position.x, e.pose.position.y, e.pose.position.z))

            #path_points = smooth_curve(path_points)

            espeleo_path.path = Polygon()
            espeleo_path.path.points = path_points

            try:
                goal = espeleo_control.msg.NavigatePathGoal()
                goal.path = espeleo_path
                mplanner.action_client_done = False
                mplanner.action_client.send_goal(goal,
                                                 feedback_cb=mplanner.action_client_feedback_callback,
                                                 done_cb=mplanner.action_client_done_callback)

                rospy.loginfo("waiting for action result...")
                while not rospy.is_shutdown() and not mplanner.action_client_done:
                    rate_slow.sleep()

                #mplanner.action_client.wait_for_result()
                action_result = mplanner.action_client.get_result()
                rospy.loginfo("action_result:%s", action_result)
            except Exception as e:
                traceback.print_exc()
                rospy.logerr('Error sending action goal %s', e.message)
            # rospy.logwarn("CONTINUE CONTINUE CONTINUE debug DEBUG DEBUG")

            rospy.loginfo("MeshPlanner very slow sleep......")
            rate_very_slow.sleep()

        except Exception as e:
            tb = traceback.format_exc()
            rospy.logerr("Main Exception: %s", str(tb))

        rate_slow.sleep()

    rospy.loginfo("MeshPlanner node stop")

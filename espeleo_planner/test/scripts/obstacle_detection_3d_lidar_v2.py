#!/usr/bin/env python

import os
import sys
import rospy
import pymesh
import rospkg
import traceback
from visualization_msgs.msg import Marker
import sensor_msgs.msg
import sensor_msgs.point_cloud2 as pc2
from scipy import spatial
from sklearn.cluster import DBSCAN
import numpy as np
import matplotlib.pyplot as plt
from mayavi import mlab

lidar_msg = None

import numpy as np
from scipy.spatial import distance
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.axes3d import get_test_data
from mpl_toolkits.mplot3d import Axes3D
import math
from math import pi
from recon_surface.srv import MeshFromPointCloud2
import pcl
import pcl.pcl_visualization
from random import randint
import time
from visualization_msgs.msg import Marker, MarkerArray

viewer = pcl.pcl_visualization.PCLVisualizering(b"3D Viewer")
viewer.InitCameraParameters()

# viewer.setCameraPosition(0, 30, 0,    0, 0, 0,   0, 0, 1)
# viewer.setCameraFieldOfView(0.523599)
# viewer.setCameraClipDistances(0.00522511, 50)
pub_closest_obstacle_pt = rospy.Publisher('/closest_obstacle_point', Marker, latch=True, queue_size=1)
pub_obstacles_pts = rospy.Publisher('/obstacles_points', MarkerArray, latch=True, queue_size=1)

color_list = []


def create_marker(pos, orientation=1.0, color=(1.0, 1.0, 1.0), m_scale=0.5, frame_id="velodyneVPL", duration=10,
                  marker_id=0, mesh_resource=None, marker_type=2, marker_text=""):
    """Create marker object using the map information and the node position

    :param pos: list of 3d postion for the marker
    :param orientation: orientation of the maker (1 for no orientation)
    :param color: a 3 vector of 0-1 rgb values
    :param m_scale: scale of the marker (1.0) for normal scale
    :param frame_id: ROS frame id
    :param duration: duration in seconds for this marker dissapearance
    :param marker_id:
    :param mesh_resource:
    :param marker_type: one of the following types (use the int value)
            http://wiki.ros.org/rviz/DisplayTypes/Marker
            ARROW = 0
            CUBE = 1
            SPHERE = 2
            CYLINDER = 3
            LINE_STRIP = 4
            LINE_LIST = 5
            CUBE_LIST = 6
            SPHERE_LIST = 7
            POINTS = 8
            TEXT_VIEW_FACING = 9
            MESH_RESOURCE = 10
            TRIANGLE_LIST = 11
    :param marker_text: text string used for the marker
    :return:
    """

    marker = Marker()
    marker.header.frame_id = frame_id
    marker.id = marker_id

    if mesh_resource:
        marker.type = marker.MESH_RESOURCE
        marker.mesh_resource = mesh_resource
    else:
        marker.type = marker_type

    marker.action = marker.ADD
    marker.scale.x = m_scale
    marker.scale.y = m_scale
    marker.scale.z = m_scale
    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.pose.orientation.w = orientation

    marker.text = marker_text

    marker.pose.position.x = pos[0]
    marker.pose.position.y = pos[1]
    marker.pose.position.z = pos[2]

    d = rospy.Duration.from_sec(duration)
    marker.lifetime = d

    return marker


def hv_in_range(x, y, z, fov, fov_type='h'):
    """
    Extract filtered in-range velodyne coordinates based on azimuth & elevation angle limit
    Args:
    `x`:velodyne points x array
    `y`:velodyne points y array
    `z`:velodyne points z array
    `fov`:a two element list, e.g.[-45,45]
    `fov_type`:the fov type, could be `h` or 'v',defualt in `h`
    Return:
    `cond`:condition of points within fov or not
    Raise:
    `NameError`:"fov type must be set between 'h' and 'v' "
    """
    d = np.sqrt(x ** 2 + y ** 2 + z ** 2)
    if fov_type == 'h':
        # print "np.arctan2(y, x):", np.arctan2(y, x)
        # print "np.deg2rad(-fov[1]):", np.deg2rad(-fov[1])
        # print "np.deg2rad(-fov[0]):", np.deg2rad(-fov[0])
        return np.logical_and(np.arctan2(y, x) > np.deg2rad(-fov[1]), np.arctan2(y, x) < np.deg2rad(-fov[0]))
    elif fov_type == 'v':
        return np.logical_and(np.arctan2(z, d) < np.deg2rad(fov[1]), np.arctan2(z, d) > np.deg2rad(fov[0]))
    else:
        raise NameError("fov type must be set between 'h' and 'v' ")


def random_color_gen():
    """ Generates a random color

        Args: None

        Returns:
            list: 3 elements, R, G, and B
    """
    r = randint(50, 255)
    g = randint(50, 255)
    b = randint(50, 255)
    return [r, g, b]


def get_color_list(cluster_count):
    global color_list
    """ Returns a list of randomized colors

        Args:
            cluster_count (int): Number of random colors to generate

        Returns:
            (list): List containing 3-element color lists
    """
    if (cluster_count > len(color_list)):
        for i in xrange(len(color_list), cluster_count):
            color_list.append(random_color_gen())
    return color_list


def get_centroid_of_pts(arr):
    length = arr.shape[0]
    sum_x = np.sum(arr[:, 0])
    sum_y = np.sum(arr[:, 1])
    sum_z = np.sum(arr[:, 2])
    return np.array([[sum_x/length, sum_y/length, sum_z/length]])


def lidar_callback(msg):
    global lidar_msg

    if lidar_msg is None:
        rospy.loginfo("lidar_callback")

    lidar_msg = msg


def find_max_list_idx(list):
    list_len = [len(i) for i in list]
    return np.argmax(np.array(list_len))


def process_lidar_msg(n_bins=72, z_std_thresh=0.1):
    global lidar_msg

    if not lidar_msg:
        return

    rospy.loginfo("process_lidar_msg")

    points = pc2.read_points_list(lidar_msg, field_names=("x", "y", "z"), skip_nans=True)

    print "size points:", len(points)
    #points = [p for p in points if p[2] < -0.39 or p[2] > -0.35 and math.sqrt(p[0] ** 2 + p[1] ** 2 + p[2] ** 2) < 3.0]
    #print "size points after Z basic cleanout:", len(points)

    #points = [p for p in points if math.sqrt(p[0] ** 2 + p[1] ** 2 + p[2] ** 2) < 3.0]
    #print "size points after distance cleanout:", len(points)

    cloud = pcl.PointCloud(np.array(points, dtype=np.float32))

    clip_distance = 2.5
    passthrough = cloud.make_passthrough_filter()
    passthrough.set_filter_field_name('x')
    passthrough.set_filter_limits(-clip_distance, clip_distance)
    cloud_filtered = passthrough.filter()

    passthrough = cloud_filtered.make_passthrough_filter()
    passthrough.set_filter_field_name('y')
    passthrough.set_filter_limits(-clip_distance, clip_distance)
    cloud_filtered = passthrough.filter()

    passthrough = cloud_filtered.make_passthrough_filter()
    passthrough.set_filter_field_name('z')
    passthrough.set_filter_limits(-clip_distance, clip_distance)
    cloud_filtered = passthrough.filter()

    vg = cloud_filtered.make_voxel_grid_filter()
    vg.set_leaf_size(0.01, 0.01, 0.01)
    cloud_filtered = vg.filter()

    # divide the pointcloud in bins
    bin_size = 360/float(n_bins)
    colors = get_color_list(n_bins)
    np_p = cloud_filtered.to_array()
    bin_idx = -1

    #viewer.InitCameraParameters()

    marker_array = MarkerArray()
    closest_p_dist = float("inf")
    closest_p = None

    for i in xrange((n_bins / 2)):
        for sign in [1, -1]:
            bin_idx += 1

            bin_start = (i * bin_size) * sign
            bin_end = ((i + 1) * bin_size) * sign

            if sign > 0:
                fov = [bin_start, bin_end]
            else:
                fov = [bin_end, bin_start]

            cond = hv_in_range(x=np_p[:, 0],
                               y=np_p[:, 1],
                               z=np_p[:, 2],
                               fov=fov,
                               fov_type='h')
            np_p_ranged = np_p[cond]

            z_std = np.std(np_p_ranged[:, 2])
            if z_std < z_std_thresh:
                cloud_cluster = pcl.PointCloud()
                cloud_cluster.from_array(np_p_ranged)
                pccolor = pcl.pcl_visualization.PointCloudColorHandleringCustom(cloud_cluster, 255, 0, 0)
                viewer.AddPointCloud_ColorHandler(cloud_cluster, pccolor, b'cluster_{}'.format(bin_idx), 0)
                print "\tz_std:", z_std
                continue

            color = colors[bin_idx]
            cloud_bin = pcl.PointCloud(np_p_ranged)

            tree = cloud_bin.make_kdtree()
            ec = cloud_bin.make_EuclideanClusterExtraction()
            ec.set_ClusterTolerance(0.15)
            ec.set_MinClusterSize(20)
            ec.set_MaxClusterSize(25000)
            ec.set_SearchMethod(tree)
            cluster_indices = ec.Extract()

            if len(cluster_indices) <= 0:
                cloud_cluster = pcl.PointCloud()
                cloud_cluster.from_array(np_p_ranged)
                pccolor = pcl.pcl_visualization.PointCloudColorHandleringCustom(cloud_cluster, 255, 0, 0)
                viewer.AddPointCloud_ColorHandler(cloud_cluster, pccolor, b'cluster_{}'.format(bin_idx), 0)
                print "\tlen(cluster_indices)", len(cluster_indices)
                continue

            max_cluster_idx = find_max_list_idx(cluster_indices)
            len_max_cluster = len(cluster_indices[max_cluster_idx])
            #print 'cluster_indices :', len(cluster_indices), " count."
            #print 'max_cluster_idx :', max_cluster_idx, " len:", len_max_cluster

            if len(cluster_indices[max_cluster_idx]) < 100:
                continue

            cluster_points = np.zeros((len_max_cluster, 3), dtype=np.float32)
            for j, indice in enumerate(cluster_indices[max_cluster_idx]):
                cluster_points[j][0] = cloud_bin[indice][0]
                cluster_points[j][1] = cloud_bin[indice][1]
                cluster_points[j][2] = cloud_bin[indice][2]

            cloud_cluster = pcl.PointCloud()
            cloud_cluster.from_array(cluster_points)
            #
            #
            pccolor = pcl.pcl_visualization.PointCloudColorHandleringCustom(cloud_cluster, color[0], color[1], color[2])
            viewer.AddPointCloud_ColorHandler(cloud_cluster, pccolor, b'cluster_{}'.format(bin_idx), 0)

            print "z_std:", z_std, "bin_start:", bin_start, "bin_end:", bin_end, "bin_idx:", bin_idx, "color:", color

            centroid = get_centroid_of_pts(cluster_points)[0]
            #print "centroid:", centroid

            x, y, z = centroid
            f_marker = create_marker((x,
                                      y,
                                      z),
                                     color=(0.6, 0.1, 0.0), duration=2, m_scale=0.25, marker_id=bin_idx)
            marker_array.markers.append(f_marker)

            d = math.sqrt(x ** 2 + y ** 2 + z ** 2)
            if d < closest_p_dist:
                closest_p_dist = d
                closest_p = centroid

            # pccolor = pcl.pcl_visualization.PointCloudColorHandleringCustom(cloud_bin, color[0], color[1], color[2])
            # viewer.AddPointCloud_ColorHandler(cloud_bin, pccolor, b'cluster_{}'.format(bin_idx), 0)

    viewer.AddCube(-0.25, 0.25, -0.15, 0.15, -0.4, -0.2, 255, 255, 255, "robot")

    # color = colors[0]
    # pccolor = pcl.pcl_visualization.PointCloudColorHandleringCustom(cloud_filtered, color[0], color[1], color[2])
    # viewer.AddPointCloud_ColorHandler(cloud_filtered, pccolor, b'cluster_{}'.format(0), 0)


    # seg = cloud_filtered.make_segmenter()
    # seg.set_optimize_coefficients(True)
    # seg.set_model_type(pcl.SACMODEL_PLANE)
    # seg.set_method_type(pcl.SAC_RANSAC)
    # seg.set_MaxIterations(100)
    # seg.set_distance_threshold(0.25)
    #
    # indices, model = seg.segment()
    # tmp = cloud_filtered.to_array()
    # tmp = np.delete(tmp, indices, 0)
    # cloud_filtered.from_array(tmp)

    # tree = cloud_filtered.make_kdtree()
    # ec = cloud_filtered.make_EuclideanClusterExtraction()
    # ec.set_ClusterTolerance(0.25)
    # ec.set_MinClusterSize(2)
    # ec.set_MaxClusterSize(25000)
    # ec.set_SearchMethod(tree)
    # cluster_indices = ec.Extract()
    #
    # print 'cluster_indices :', len(cluster_indices), " count."
    #
    # colors = get_color_list(len(cluster_indices))
    #
    # cloud_cluster = pcl.PointCloud()
    #
    # for j, indices in enumerate(cluster_indices):
    #     # cloudsize = indices
    #     print 'j:', j, 'indices:', str(len(indices))
    #
    #     points = np.zeros((len(indices), 3), dtype=np.float32)
    #
    #     for i, indice in enumerate(indices):
    #         points[i][0] = cloud_filtered[indice][0]
    #         points[i][1] = cloud_filtered[indice][1]
    #         points[i][2] = cloud_filtered[indice][2]
    #
    #     z_std = np.std(points[:, 2])
    #     print "z std:", z_std
    #
    #     cloud_cluster.from_array(points)
    #     ss = "/tmp/cluster/cloud_cluster_" + str(j) + ".pcd"
    #     pcl.save(cloud_cluster, ss)
    #
    #     color = colors[j]
    #     pccolor = pcl.pcl_visualization.PointCloudColorHandleringCustom(cloud, color[0], color[1], color[2])
    #     viewer.AddPointCloud_ColorHandler(cloud_cluster, pccolor, b'cluster_{}'.format(j), 0)

    if closest_p is not None:
        closest_p_marker = create_marker((closest_p[0],
                                      closest_p[1],
                                      closest_p[2]),
                                     color=(0.9, 0.1, 0.0), duration=2, m_scale=0.5, marker_id=0)
        pub_closest_obstacle_pt.publish(closest_p_marker)
        pub_obstacles_pts.publish(marker_array)

    v = True
    while v:
        v = not (viewer.WasStopped())
        viewer.SpinOnce()
        #time.sleep(0.5)
        #break

    # for j, indices in enumerate(cluster_indices):
    #     viewer.RemovePointCloud(b'cluster_{}'.format(j), 0)

    # for i in xrange(n_bins):
    #     viewer.RemovePointCloud(b'cluster_{}'.format(i), 0)

    viewer.remove_all_pointclouds()
    viewer.remove_all_shapes()


if __name__ == '__main__':
    rospy.init_node('obstacle_detection_3d_lidar')
    rospy.loginfo("init node...")

    rospy.Subscriber('/velodyne/points2', sensor_msgs.msg.PointCloud2, lidar_callback)
    rate_slow = rospy.Rate(1.0)

    while not rospy.is_shutdown():
        try:
            process_lidar_msg()
        except Exception as e:
            tb = traceback.format_exc()
            rospy.logerr("Main Exception: %s", str(tb))

        rate_slow.sleep()

    rospy.loginfo("obstacle_detection_3d_lidar node stop")
3
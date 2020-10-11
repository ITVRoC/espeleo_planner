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

import sys
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
import matplotlib
import timeit
from geometry_msgs.msg import Twist, Pose, Point


viewer = pcl.pcl_visualization.PCLVisualizering(b"3D Viewer")
viewer.InitCameraParameters()

# viewer.setCameraPosition(0, 30, 0,    0, 0, 0,   0, 0, 1)
# viewer.setCameraFieldOfView(0.523599)
# viewer.setCameraClipDistances(0.00522511, 50)
pub_closest_obstacle_marker = rospy.Publisher('/closest_obstacle_marker', Marker, latch=True, queue_size=1)
pub_obstacles_pts = rospy.Publisher('/obstacles_points_markers', MarkerArray, latch=True, queue_size=1)
pub_closest_obstacle_pt = rospy.Publisher('/closest_obstacle_point', Point, latch=True, queue_size=1)

color_list = []


def create_marker(pos, orientation=1.0, color=(1.0, 1.0, 1.0), m_scale=0.5, frame_id="/velodyneVPL", duration=10,
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


def jetmap_interpolate(val, y0, x0, y1, x1):
    return (val-x0)*(y1-y0)/(x1-x0) + y0


def jetmap_color_base(val):
    if val <= -0.75:
        return 0
    elif val <= -0.25:
        return jetmap_interpolate(val, 0.0, -0.75, 1.0, -0.25)
    elif val <= 0.25:
        return 1.0
    elif val <= 0.75:
        return jetmap_interpolate(val, 1.0, 0.25, 0.0, 0.75 )
    else:
        return 0.0


def jetmap_red(gray):
    return int(jetmap_color_base(gray - 0.5) * 255)


def jetmap_green(gray):
    return int(jetmap_color_base(gray) * 255)


def jetmap_blue(gray):
    return int(jetmap_color_base(gray + 0.5) * 255)


def get_color_list(cluster_count, is_new_list=False):
    global color_list
    """ Returns a list of randomized colors

        Args:
            cluster_count (int): Number of random colors to generate

        Returns:
            (list): List containing 3-element color lists
    """

    min_v = 0
    max_v = cluster_count
    cmap = matplotlib.cm.get_cmap('tab20')

    """
    Accent, Accent_r, Blues, Blues_r, BrBG, BrBG_r, BuGn, BuGn_r, BuPu, BuPu_r, CMRmap, CMRmap_r, Dark2, Dark2_r, GnBu, 
    GnBu_r, Greens, Greens_r, Greys, Greys_r, OrRd, OrRd_r, Oranges, Oranges_r, PRGn, PRGn_r, Paired, Paired_r, 
    Pastel1, Pastel1_r, Pastel2, Pastel2_r, PiYG, PiYG_r, PuBu, PuBuGn, PuBuGn_r, PuBu_r, PuOr, PuOr_r, PuRd, PuRd_r, 
    Purples, Purples_r, RdBu, RdBu_r, RdGy, RdGy_r, RdPu, RdPu_r, RdYlBu, RdYlBu_r, RdYlGn, RdYlGn_r, Reds, Reds_r, 
    Set1, Set1_r, Set2, Set2_r, Set3, Set3_r, Spectral, Spectral_r, Wistia, Wistia_r, YlGn, YlGnBu, YlGnBu_r, YlGn_r, 
    YlOrBr, YlOrBr_r, YlOrRd, YlOrRd_r, afmhot, afmhot_r, autumn, autumn_r, binary, binary_r, bone, bone_r, brg, brg_r, 
    bwr, bwr_r, cividis, cividis_r, cool, cool_r, coolwarm, coolwarm_r, copper, copper_r, cubehelix, cubehelix_r, flag, 
    flag_r, gist_earth, gist_earth_r, gist_gray, gist_gray_r, gist_heat, gist_heat_r, gist_ncar, gist_ncar_r, 
    gist_rainbow, gist_rainbow_r, gist_stern, gist_stern_r, gist_yarg, gist_yarg_r, gnuplot, gnuplot2, gnuplot2_r, 
    gnuplot_r, gray, gray_r, hot, hot_r, hsv, hsv_r, inferno, inferno_r, jet, jet_r, magma, magma_r, nipy_spectral, 
    nipy_spectral_r, ocean, ocean_r, pink, pink_r, plasma, plasma_r, prism, prism_r, rainbow, rainbow_r, seismic, 
    seismic_r, spring, spring_r, summer, summer_r, tab10, tab10_r, tab20, tab20_r, tab20b, tab20b_r, tab20c, tab20c_r, 
    terrain, terrain_r, viridis, viridis_r, winter, winter_r
    """

    if cluster_count != len(color_list) or is_new_list:
        color_list = []
        for i in xrange(cluster_count):
            normalized_v = (i - min_v) / float(max_v - min_v)
            color = cmap(normalized_v)
            rgb_color = (color[0] * 255, color[1] * 255, color[2] * 255)
            color_list.append(rgb_color)
            #color_list.append((jetmap_red(jet_v), jetmap_green(jet_v), jetmap_blue(jet_v)))
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


def process_lidar_msg(is_plot=True):
    global lidar_msg

    if not lidar_msg:
        return

    rospy.loginfo("process_lidar_msg")

    points = pc2.read_points_list(lidar_msg, field_names=("x", "y", "z"), skip_nans=True)

    print "size points:", len(points)

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
    n_bins = 72
    bin_size = 360/float(n_bins)
    np_p = cloud_filtered.to_array()
    bin_idx = -1

    marker_array = MarkerArray()
    closest_p_dist = float("inf")
    closest_p = None

    cloud_binned = pcl.PointCloud()

    bin_pairs = []
    for i in xrange((n_bins / 2)):
        for sign in [1, -1]:
            bin_start = (i * bin_size) * sign
            bin_end = ((i + 1) * bin_size) * sign

            if sign > 0:
                fov = [bin_start, bin_end]
            else:
                fov = [bin_end, bin_start]

            bin_pairs.append(fov)

    for i, fov in enumerate(bin_pairs):
        cond = hv_in_range(x=np_p[:, 0],
                           y=np_p[:, 1],
                           z=np_p[:, 2],
                           fov=fov,
                           fov_type='h')
        np_p_ranged = np_p[cond]

        Z = np_p_ranged[:, 2]
        if Z.shape[0] <= 1:
            continue

        z_std = np.std(Z)
        if z_std <= 0.1:
            continue

        if cloud_binned.size <= 0:
            cloud_binned.from_array(np_p_ranged)
        else:
            a = np.asarray(cloud_binned)
            a = np.concatenate((a, np_p_ranged))
            cloud_binned.from_array(a)

    if is_plot:
        pccolor = pcl.pcl_visualization.PointCloudColorHandleringCustom(cloud_binned, 255, 255, 255)
        viewer.AddPointCloud_ColorHandler(cloud_binned, pccolor, b'z_std_filtering', 0)

    #pcl.save(cloud_cluster, "/tmp/pcloud.test.ply")

    tree = cloud_binned.make_kdtree()
    ec = cloud_binned.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.10)
    ec.set_MinClusterSize(5)
    ec.set_MaxClusterSize(25000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    n_clusters = len(cluster_indices)

    if n_clusters <= 0:
        rospy.logerr("n_clusters <= 0")
        return

    #print "n_clusters:", n_clusters

    # clustering point cloud
    cloud_cluster_concat = pcl.PointCloud()
    for i in xrange(n_clusters):
        cluster = cluster_indices[i]
        cluster_size = len(cluster)

        cluster_points = np.zeros((cluster_size, 3), dtype=np.float32)
        for j, idx in enumerate(cluster):
            cluster_points[j][0] = cloud_binned[idx][0]
            cluster_points[j][1] = cloud_binned[idx][1]
            cluster_points[j][2] = cloud_binned[idx][2]

        z_std = np.std(cluster_points[:, 2])
        if z_std <= 0.1:
            continue

        if cloud_cluster_concat.size <= 0:
            cloud_cluster_concat.from_array(cluster_points)
        else:
            a = np.asarray(cloud_cluster_concat)
            a = np.concatenate((a, cluster_points))
            cloud_cluster_concat.from_array(a)

    if cloud_cluster_concat.size <= 0:
        rospy.logwarn("cloud_cluster_concat size <= 0")
        return

    # get the obstacles by angle FOV
    colors = get_color_list(len(bin_pairs))
    cloud_points = np.asarray(cloud_cluster_concat)
    for i, fov in enumerate(bin_pairs):
        cond = hv_in_range(x=cloud_points[:, 0],
                           y=cloud_points[:, 1],
                           z=cloud_points[:, 2],
                           fov=fov,
                           fov_type='h')
        np_p_ranged = cloud_points[cond]

        fov_cloud = pcl.PointCloud()
        fov_cloud.from_array(np_p_ranged)

        if is_plot:
            color = colors[i]
            pccolor = pcl.pcl_visualization.PointCloudColorHandleringCustom(fov_cloud, color[0], color[1], color[2])
            viewer.AddPointCloud_ColorHandler(fov_cloud, pccolor, b'fov_cloud_{}'.format(i), 0)

        centroid = get_centroid_of_pts(np_p_ranged)[0]
        x, y, z = centroid
        f_marker = create_marker((x,
                                  y,
                                  z),
                                 color=(0.6, 0.1, 0.0), duration=2, m_scale=0.25, marker_id=i)
        marker_array.markers.append(f_marker)

        d = math.sqrt(x ** 2 + y ** 2 + z ** 2)
        if d < closest_p_dist:
            closest_p_dist = d
            closest_p = centroid

    if closest_p is not None:
        closest_p_marker = create_marker((closest_p[0],
                                      closest_p[1],
                                      closest_p[2]),
                                     color=(0.9, 0.1, 0.0), duration=2, m_scale=0.5, marker_id=0)
        pub_closest_obstacle_marker.publish(closest_p_marker)
        pub_closest_obstacle_pt.publish(Point(x=closest_p[0], y=closest_p[1], z=closest_p[2]))
        pub_obstacles_pts.publish(marker_array)

    if is_plot:
        viewer.AddCube(-0.25, 0.25, -0.15, 0.15, -0.4, -0.2, 255, 255, 255, "robot")

        viewer.SpinOnce()

        # v = True
        # while v:
        #     v = not (viewer.WasStopped())
        #     viewer.SpinOnce()
        #     # time.sleep(0.5)
        #     break

        viewer.RemoveShape("robot", 0)
        viewer.remove_all_pointclouds()
        viewer.remove_all_shapes()
        # sys.exit(0)


if __name__ == '__main__':
    rospy.init_node('obstacle_detection_3d_lidar')
    rospy.loginfo("init node...")

    rospy.Subscriber('/velodyne/points2', sensor_msgs.msg.PointCloud2, lidar_callback)
    rate_slow = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            time1 = time.time()
            process_lidar_msg()
            time2 = time.time()
            print 'process_lidar_msg %0.3f ms' % ((time2 - time1) * 1000.0)
        except Exception as e:
            tb = traceback.format_exc()
            rospy.logerr("Main Exception: %s", str(tb))

        rate_slow.sleep()

    rospy.loginfo("obstacle_detection_3d_lidar node stop")

#!/usr/bin/env python

import time
import rospy
import traceback
import pymesh
from recon_surface.srv import MeshFromPointCloud2
from visualization_msgs.msg import MarkerArray, Marker
import random
import numpy as np

import espeleo_control.msg
import std_msgs.msg
from geometry_msgs.msg import Polygon, Point32, Point
import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

from pyquaternion import Quaternion

import pcl
import pcl.pcl_visualization

import rospy
import pcl
import numpy as np
import ctypes
import struct
import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from random import randint
import re
import time
import traceback
from ctypes import *

import espeleo_control.msg
import numpy as np
import open3d as o3d
import rospy
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg
from geometry_msgs.msg import Polygon, Point32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sklearn.neighbors import KDTree
from visualization_msgs.msg import MarkerArray, Marker

map_pointcloud_msg = None
odom_msg = None


def map_point_cloud_callback(msg):
    global map_pointcloud_msg
    map_pointcloud_msg = msg


def odom_callback(msg):
    global odom_msg
    odom_msg = msg


def ros_to_pcl(ros_cloud):
    """ Converts a ROS PointCloud2 message to a pcl PointXYZRGB

        Args:
            ros_cloud (PointCloud2): ROS PointCloud2 message

        Returns:
            pcl.PointCloud_PointXYZRGB: PCL XYZRGB point cloud
    """
    points_list = []

    for data in pc2.read_points(ros_cloud, skip_nans=True):
        points_list.append([data[0], data[1], data[2], data[3]])

    pcl_data = pcl.PointCloud_PointXYZRGB()
    pcl_data.from_list(points_list)

    return pcl_data


def pcl_to_ros(pcl_array):
    """ Converts a ROS PointCloud2 message to a pcl PointXYZRGB

        Args:
            pcl_array (PointCloud_PointXYZRGB): A PCL XYZRGB point cloud

        Returns:
            PointCloud2: A ROS point cloud
    """
    ros_msg = PointCloud2()

    ros_msg.header.stamp = rospy.Time.now()
    ros_msg.header.frame_id = "os1_init"

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

def convert_cloud_from_ros_to_open3d(ros_cloud):
    # Bit operations
    convert_rgbUint32_to_tuple = lambda rgb_uint32: (
        (rgb_uint32 & 0x00ff0000) >> 16, (rgb_uint32 & 0x0000ff00) >> 8, (rgb_uint32 & 0x000000ff)
    )
    convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
        int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
    )

    # Get cloud data from ros_cloud
    field_names = [field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names=field_names))

    # Check empty
    open3d_cloud = o3d.geometry.PointCloud()
    if len(cloud_data) == 0:
        return None

    # Set open3d_cloud
    if "rgb" in field_names:
        IDX_RGB_IN_FIELD = 3  # x, y, z, rgb

        # Get xyz
        xyz = [(x, y, z) for x, y, z, rgb in cloud_data]  # (why cannot put this line below rgb?)

        # Get rgb
        # Check whether int or float
        if type(cloud_data[0][IDX_RGB_IN_FIELD]) == float:  # if float (from pcl::toROSMsg)
            rgb = [convert_rgbFloat_to_tuple(rgb) for x, y, z, rgb in cloud_data]
        else:
            rgb = [convert_rgbUint32_to_tuple(rgb) for x, y, z, rgb in cloud_data]

        # combine
        open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))
        open3d_cloud.colors = o3d.utility.Vector3dVector(np.array(rgb) / 255.0)
    else:
        xyz = [(x, y, z) for x, y, z, _ in cloud_data]  # get xyz
        open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))

    return open3d_cloud


def prepare_pointcloud(curr_lidar_pcloud):
    # LiDAR pointcloud
    pcd = convert_cloud_from_ros_to_open3d(curr_lidar_pcloud)

    # pcd.estimate_normals(
    #     search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=48))

    # np_points = np.asarray(pcd.points)
    # np_normals = np.asarray(pcd.normals)
    #
    # normalized_np_normals = (np_normals / np.linalg.norm(np_normals, axis=1)[:, None])
    # clipped_u = np.clip(np.dot(normalized_np_normals, [0, 0, -1]), -1.0, 1.0)
    # arccos_u = np.arccos(clipped_u)
    # degrees_u = np.degrees(arccos_u)
    # degrees_u = np.where(~(degrees_u > 90), degrees_u, np.abs(degrees_u - 180))
    #
    # np_points = np_points[(degrees_u <= 18)]
    # np_normals = np_normals[(degrees_u <= 18)]
    #
    # filtered_downpcd = o3d.geometry.PointCloud()
    # filtered_downpcd.points = o3d.utility.Vector3dVector(np_points)
    # filtered_downpcd.normals = o3d.utility.Vector3dVector(np_normals)
    #
    # o3d.io.write_point_cloud("/tmp/test_pointcloud_filtered_downpcd.ply", filtered_downpcd, print_progress=True)
    #
    # return filtered_downpcd


if __name__ == '__main__':
    rospy.init_node('publish_frontiers_node')

    rospy.loginfo("publish_frontiers_node start")

    rate_very_slow = rospy.Rate(0.05)
    rate_slow = rospy.Rate(1.0)
    rate_fast = rospy.Rate(10.0)

    # subscribers
    #rospy.Subscriber('/laser_cloud_surround2', PointCloud2, map_point_cloud_callback)
    #rospy.Subscriber('/full_cloud_projected2', PointCloud2, map_point_cloud_callback)
    # rospy.Subscriber('/octomap_point_cloud_centers', PointCloud2, map_point_cloud_callback)
    rospy.Subscriber('/full_cloud_projected2', PointCloud2, map_point_cloud_callback)
    rospy.Subscriber('/integrated_to_init2', Odometry, odom_callback)

    rospy.sleep(3.)

    while not rospy.is_shutdown():
        try:
            # check if the source and target are valid
            if not map_pointcloud_msg or not odom_msg:
                rospy.loginfo("Mesh frontier estimation waiting for data... %s %s", str(type(map_pointcloud_msg)), str(type(odom_msg)))
                rate_slow.sleep()
                continue

            curr_map = map_pointcloud_msg
            src = odom_msg

            map_pointcloud_msg = None
            odom_msg = None

            robot_pos = (src.pose.pose.position.x, src.pose.pose.position.y, src.pose.pose.position.z)
            rate_fast.sleep()  # allow publishing the markers without waiting to the last rate.sleep()

            points = pc2.read_points_list(curr_map, field_names=("x", "y", "z"), skip_nans=True)
            cloud = pcl.PointCloud(np.array(points, dtype=np.float32))
            # cloud = ros_to_pcl(curr_map)

            clip_distance = 5
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

            #new_cloud_msg = pc2.create_cloud(curr_map.header, curr_map.fields, cloud_filtered.points)
            new_cloud_msg = pcl_to_ros(cloud_filtered)

            #prepare_pointcloud(new_cloud_msg)

            # run the service to convert the point cloud to mesh
            mesh_filepath = None
            try:
                rospy.wait_for_service('/mesh_from_pointclouds', timeout=3)
                mesh_from_pointcloud = rospy.ServiceProxy('/mesh_from_pointclouds', MeshFromPointCloud2)
                mesh_src_point = Point(0.0, 0.0, 0.0)
                resp1 = mesh_from_pointcloud(new_cloud_msg, mesh_src_point)
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

            rate_slow.sleep()
        except Exception as e:
            tb = traceback.format_exc()
            rospy.logerr("Main Exception: %s", str(tb))

        rate_slow.sleep()

    rospy.loginfo("MeshPlanner node stop")

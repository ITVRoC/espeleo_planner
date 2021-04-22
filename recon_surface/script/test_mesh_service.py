#!/usr/bin/env python

import rospy
import sensor_msgs
from recon_surface.srv import MeshFromPointCloud2
import sys
import time
import os
import pcl
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import ctypes
import struct
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Point
import traceback


# Database of small test mesh PLY files
# https://people.sc.fsu.edu/~jburkardt/data/ply/ply.html


def pcl_to_ros(pcl_array, frame_id="world"):
    """ Converts a ROS PointCloud2 message to a pcl PointXYZRGB

        Args:test_mesh_service.py
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


def pointcloud2_callback(msg):
    global processing_service
    rospy.loginfo("pointcloud2_callback called")

    try:
        rospy.wait_for_service('/mesh_from_pointclouds', timeout=3)
        mesh_from_pointcloud = rospy.ServiceProxy('/mesh_from_pointclouds', MeshFromPointCloud2)

        points = pc2.read_points_list(msg, field_names=("x", "y", "z"), skip_nans=True)
        cloud = pcl.PointCloud(np.array(points, dtype=np.float32))

        clip_distance = 20
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

        filtered_msg = pcl_to_ros(cloud_filtered, frame_id=msg.header.frame_id)

        time1 = time.time()
        mesh_src_point = Point(0.0, 0.0, 0.0)
        resp1 = mesh_from_pointcloud(filtered_msg, mesh_src_point)
        time2 = time.time()

        rospy.loginfo("pointcloud processed result: %s", resp1)
        rospy.loginfo("service executed in %f seconds", (time2-time1))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        processing_service = False
    except Exception as e:
        rospy.logerr("Exception: %s", e)
        processing_service = False
        traceback.print_exc()

    # os.system("rosnode kill read_stl_node");
    # rospy.signal_shutdown(0)
    # sys.exit()


if __name__ == "__main__":
    rospy.init_node("test_load_stl_call_recon_surface_service", anonymous=False)

    scan_sub = rospy.Subscriber("/test_point_cloud", sensor_msgs.msg.PointCloud2, pointcloud2_callback)
    rospy.spin()
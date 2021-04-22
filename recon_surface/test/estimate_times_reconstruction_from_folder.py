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


def run_test(folder):
    rospy.loginfo("pointcloud2_callback called")

    ply_file_paths = []

    for file in os.listdir(folder):
        if file.endswith(".ply"):
            ply_file_paths.append(os.path.join(folder, file))
            print len(ply_file_paths), ply_file_paths[-1]

    ply_file_paths.pop(0)
    ply_file_paths.pop(0)

    for f_path in ply_file_paths:
        rospy.wait_for_service('/mesh_from_pointclouds', timeout=3)

        print "loading cloud...", f_path
        pcl_cloud = pcl.load(f_path)
        print "cloud shape:", pcl_cloud.size

        cloud = pcl.PointCloud(np.array(pcl_cloud, dtype=np.float32))
        filtered_msg = pcl_to_ros(cloud)

        mesh_src_point = Point(0, 0, 0)

        for i in xrange(3):
            try:
                mesh_from_pointcloud = rospy.ServiceProxy('/mesh_from_pointclouds', MeshFromPointCloud2)
                time1 = time.time()
                print "mesh_from_pointcloud...",
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
    rospy.loginfo("Initializing test...")
    time.sleep(2)
    run_test("/media/h3ct0r/f59508e7-022c-4055-bdac-217324eaf1af/home/h3ct0r/Desktop/bag_ufmg_multi_level_22_12_2020/pclouds/")
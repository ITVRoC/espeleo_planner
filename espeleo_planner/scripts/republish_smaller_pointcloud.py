#!/usr/bin/env python

import rospy
import espeleo_control.msg
import std_msgs.msg
from geometry_msgs.msg import Polygon, Point32
import nav_msgs.msg

import pcl
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import ctypes
import struct
from sensor_msgs.msg import PointCloud2, PointField
import traceback
from nav_msgs.msg import Odometry

class RepublishPointCloud:
    """This node converts the normal polygon path to a Espeleorobo path message
    """

    def __init__(self):
        self.odom_msg = None
        self.map_pointcloud_msg = None

        self.pointcloud_pub = None
        self.init_node()

    def init_node(self):
        """Init node with ROS bindings
        :return:
        """

        self.pointcloud_pub = rospy.Publisher('/laser_cloud_surround2_small', PointCloud2, queue_size=1)

        rospy.Subscriber('/laser_cloud_surround2', PointCloud2, self.map_point_cloud_callback)
        rospy.Subscriber('/integrated_to_init2', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        self.odom_msg = msg

    def map_point_cloud_callback(self, msg):
        self.map_pointcloud_msg = msg

    def republish_cloud(self):
        if not self.odom_msg:
            return

        if not self.map_pointcloud_msg:
            return

        filtered_pcloud = RepublishPointCloud.clip_pointcloud_msg(self.map_pointcloud_msg,
                                                                  self.odom_msg,
                                                                  clip_distance=15)
        self.pointcloud_pub.publish(filtered_pcloud)

    @staticmethod
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

    @staticmethod
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
        passthrough.set_filter_limits(-clip_distance + src.pose.pose.position.x,
                                      clip_distance + src.pose.pose.position.x)
        cloud_filtered = passthrough.filter()

        passthrough = cloud_filtered.make_passthrough_filter()
        passthrough.set_filter_field_name('y')
        passthrough.set_filter_limits(-clip_distance + src.pose.pose.position.y,
                                      clip_distance + src.pose.pose.position.y)
        cloud_filtered = passthrough.filter()

        passthrough = cloud_filtered.make_passthrough_filter()
        passthrough.set_filter_field_name('z')
        passthrough.set_filter_limits(-clip_distance + src.pose.pose.position.z,
                                      clip_distance + src.pose.pose.position.z)
        cloud_filtered = passthrough.filter()

        vg = cloud_filtered.make_voxel_grid_filter()
        vg.set_leaf_size(0.1, 0.1, 0.1)
        cloud_filtered = vg.filter()

        filtered_msg = RepublishPointCloud.pcl_to_ros(cloud_filtered, frame_id=msg.header.frame_id)
        return filtered_msg


if __name__ == '__main__':
    rospy.init_node('cloud_republish_espeleo', anonymous=True)
    rospy.loginfo("cloud_republish_espeleo node start")

    pcloud_repub = RepublishPointCloud()

    rate_slow = rospy.Rate(1.0)

    while not rospy.is_shutdown():
        pcloud_repub.republish_cloud()
        rate_slow.sleep()

    rospy.loginfo("cloud_republish_espeleo node stop")

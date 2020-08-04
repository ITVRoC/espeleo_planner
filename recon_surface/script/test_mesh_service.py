#!/usr/bin/env python

import rospy
import sensor_msgs
from recon_surface.srv import MeshFromPointCloud2
import sys
import time
import os


# Database of small test mesh PLY files
# https://people.sc.fsu.edu/~jburkardt/data/ply/ply.html


def pointcloud2_callback(msg):
    global processing_service
    rospy.loginfo("pointcloud2_callback called")

    try:
        rospy.wait_for_service('/mesh_from_pointclouds', timeout=3)
        mesh_from_pointcloud = rospy.ServiceProxy('/mesh_from_pointclouds', MeshFromPointCloud2)

        time1 = time.time()
        resp1 = mesh_from_pointcloud(msg)
        time2 = time.time()

        rospy.loginfo("pointcloud processed result: %s", resp1)
        rospy.loginfo("service executed in %f seconds", (time2-time1))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        processing_service = False
    except Exception as e:
        rospy.logerr("Exception: %s", e)
        processing_service = False

    os.system("rosnode kill read_stl_node");
    rospy.signal_shutdown(0)
    sys.exit()

if __name__ == "__main__":
    rospy.init_node("test_load_stl_call_recon_surface_service", anonymous = False)

    scan_sub = rospy.Subscriber("/test_point_cloud", sensor_msgs.msg.PointCloud2, pointcloud2_callback)
    rospy.spin()
#!/usr/bin/env python

import rospy
import traceback

from sympy.codegen.fnodes import isign
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import re

import espeleo_control.msg
import std_msgs.msg
from geometry_msgs.msg import Polygon, Point32, Point
from mesh_planner import mesh_helper, graph_metrics, mesh_planner_node, pointcloud_planner_base
from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
from ctypes import *
import time
from sklearn.neighbors import KDTree

from pyquaternion import Quaternion


map_pointcloud_msg = None
odom_msg = None
mi_markers = MarkerArray()
mesh_marker = Marker()
numeric_const_pattern = '[-+]? (?: (?: \d* \. \d+ ) | (?: \d+ \.? ) )(?: [Ee] [+-]? \d+ ) ?'
number_rx = re.compile(numeric_const_pattern, re.VERBOSE)
sampled_pc = None


def map_point_cloud_callback(msg):
    global map_pointcloud_msg
    map_pointcloud_msg = msg


def odom_callback(msg):
    global odom_msg
    odom_msg = msg


def mi_markers_callback(msg):
    global mi_markers
    mi_markers = msg


def mesh_marker_callback(msg):
    global mesh_marker, sampled_pc
    mesh_marker = msg

    mesh = o3d.io.read_triangle_mesh(mesh_marker.mesh_resource[7:])
    sampled_pc = mesh.sample_points_uniformly(number_of_points=len(mesh.vertices))


# The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

# Bit operations
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8
convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
)
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)


def convert_cloud_from_ros_to_open3d(ros_cloud):
    # Get cloud data from ros_cloud
    field_names = [field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names=field_names))

    # Check empty
    open3d_cloud = o3d.geometry.PointCloud()
    if len(cloud_data) == 0:
        print("Converting an empty cloud")
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
        open3d_cloud.points = o3d.Vector3dVector(np.array(xyz))
        open3d_cloud.colors = o3d.Vector3dVector(np.array(rgb) / 255.0)
    else:
        xyz = [(x, y, z) for x, y, z, _ in cloud_data]  # get xyz
        open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))

    # return
    return open3d_cloud


if __name__ == '__main__':
    rospy.init_node('pointcloud_planner_exploration_rrt_main_node')

    rospy.loginfo("pointcloud node start")

    mplanner = mesh_planner_node.MeshPlannerNode()
    mplanner.init_action_node()

    rate_very_slow = rospy.Rate(0.05)
    rate_slow = rospy.Rate(1.0)
    rate_fast = rospy.Rate(10.0)

    # subscribers
    rospy.Subscriber('/laser_cloud_surround2', PointCloud2, map_point_cloud_callback)
    rospy.Subscriber('/integrated_to_init2', Odometry, odom_callback)
    rospy.Subscriber('/frontiers_mutual_information', MarkerArray, mi_markers_callback)
    rospy.Subscriber('/reconstructed_mesh_marker_normal', Marker, mesh_marker_callback)

    # publishers
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.sleep(3.)

    # move robot front and back to generate first point cloud
    twist_msg = Twist()
    twist_msg.linear.x = 1.0
    for i in xrange(50):
        cmd_vel_pub.publish(twist_msg)
        rospy.sleep(0.2)

    twist_msg.linear.x = -1.0
    for i in xrange(40):
        cmd_vel_pub.publish(twist_msg)
        rospy.sleep(0.2)

    twist_msg.linear.x = 0.0
    for i in xrange(30):
        cmd_vel_pub.publish(twist_msg)
        rospy.sleep(0.2)

    error_counter = 0
    while not rospy.is_shutdown():
        try:
            if error_counter > 5:
                rospy.logerr("error_counter > 5, stopping exploration...")
                rate_slow.sleep()
                break

            # check if the source and target are valid
            if not map_pointcloud_msg or \
                    not odom_msg or len(mi_markers.markers) <= 0 or \
                    not mesh_marker or not sampled_pc:
                rospy.loginfo("Waiting for data for point cloud RRT...")
                rate_slow.sleep()
                continue

            curr_pcloud = map_pointcloud_msg
            src = odom_msg
            curr_mi_markers = mi_markers

            #map_pointcloud_msg = None
            odom_msg = None

            rospy.loginfo("Start planning...")

            # LiDAR pointcloud
            start_time = time.clock()
            pcd = convert_cloud_from_ros_to_open3d(curr_pcloud)
            p1_load = np.asarray(pcd.points)
            p2_load = np.asarray(sampled_pc.points)
            pcd.points = o3d.utility.Vector3dVector(np.concatenate((p1_load, p2_load), axis=0))

            downpcd_simple = pcd.voxel_down_sample(voxel_size=0.3)
            downpcd_simple.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=48))
            o3d.io.write_point_cloud("/tmp/test_pointcloud.ply", downpcd_simple, print_progress=True)

            np_points = np.asarray(downpcd_simple.points)
            np_normals = np.asarray(downpcd_simple.normals)

            print("np_normals:", np_normals.shape)

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

            # ply_filepath = "/tmp/downpcd_filtered_planning.ply"
            # o3d.io.write_point_cloud(ply_filepath, downpcd)
            print("filtered_points:", len(filtered_points), "time processing pcloud:", (time.clock() - start_time))

            src_marker = mesh_helper.create_marker((src.pose.pose.position.x,
                                                    src.pose.pose.position.y,
                                                    src.pose.pose.position.z),
                                                   color=(0.0, 1.0, 0.0), duration=60, marker_id=0)
            mplanner.pub_src_point.publish(src_marker)
            rate_fast.sleep()  # allow publishing the markers without waiting to the last rate.sleep()

            start_time = time.clock()
            target_faces = []
            target_mi = []
            kd_tree_of_lines = KDTree(np.array(filtered_points), leaf_size=2)

            for marker in curr_mi_markers.markers:
                pos = (marker.pose.position.x, marker.pose.position.y, marker.pose.position.z - 1.0)

                dist, idx = kd_tree_of_lines.query([pos], k=1)
                target_faces.append(idx[0][0])

                numbers = number_rx.findall(marker.text)
                if len(numbers) > 0:
                    target_mi.append(float(numbers[0]))
                else:
                    target_mi.append(0.0)
                    print("ERROR MI CANNOT BE EXTRACTED FROM:", marker.text)

            target_mi = np.array(target_mi)
            normalized_mi = target_mi / target_mi.sum()

            print "target_faces:", target_faces
            print "normalized_mi:", normalized_mi
            print "time processing mi faces:", (time.clock() - start_time)

            start_time = time.clock()
            graph_metric_types = [graph_metrics.GraphMetricType.COMBINED]
            planner = pointcloud_planner_base.PointCloudPlannerBase(filtered_downpcd, graph_metric_types)
            print "time constructor PointCloudPlannerBase:", (time.clock() - start_time)

            source_face_idx = mesh_helper.find_closer_centroid(filtered_points,
                                                               (src.pose.pose.position.x,
                                                                src.pose.pose.position.y,
                                                                src.pose.pose.position.z), force_return_closer=True)
            return_dict = planner.run(source_face_idx, target_faces, list(normalized_mi), is_debug=False)
            #print("return_dict:", return_dict)

            mplanner.publish_paths(return_dict)

            path_msg = return_dict[graph_metrics.GraphMetricType.COMBINED]['path_msg']

            espeleo_path = espeleo_control.msg.Path()
            espeleo_path.header = std_msgs.msg.Header()
            espeleo_path.header.stamp = rospy.Time.now()
            espeleo_path.header.frame_id = path_msg.header.frame_id
            espeleo_path.closed_path_flag = False
            espeleo_path.insert_n_points = 5
            espeleo_path.filter_path_n_average = 5

            path_points = []
            for e in path_msg.poses:
                path_points.append(Point32(e.pose.position.x, e.pose.position.y, e.pose.position.z))

            espeleo_path.path = Polygon()
            espeleo_path.path.points = path_points

            try:
                goal = espeleo_control.msg.NavigatePathGoal()
                goal.path = espeleo_path
                mplanner.action_client_done = False
                mplanner.action_client.send_goal(goal,
                                                 feedback_cb=mplanner.action_client_feedback_callback,
                                                 done_cb=mplanner.action_client_done_callback)

                #rospy.loginfo("waiting for action result...")
                # while not rospy.is_shutdown() and not mplanner.action_client_done:
                #
                #     rate_slow.sleep()
                #
                # #mplanner.action_client.wait_for_result()
                # action_result = mplanner.action_client.get_result()
                # rospy.loginfo("action_result:%s", action_result)

                #rospy.loginfo("Very slow sleep after sending the path...")
                #rate_very_slow.sleep()
            except Exception as e:
                traceback.print_exc()
                rospy.logerr('Error sending action goal %s', e.message)

            rospy.loginfo("RRTPlanner slow sleep...")
            rate_slow.sleep()
        except Exception as e:
            tb = traceback.format_exc()
            rospy.logerr("Main Exception: %s", str(tb))

        rate_slow.sleep()

    rospy.loginfo("MeshPlanner node stop")

#!/usr/bin/env python

from pydoc import describe
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
from std_msgs.msg import Header
from geometry_msgs.msg import Polygon, Point32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from sklearn.neighbors import KDTree
import scipy
from visualization_msgs.msg import MarkerArray, Marker
from scipy.spatial.distance import cdist
from tqdm import tqdm

from mesh_planner import mesh_helper, graph_metrics, mesh_planner_node, pointcloud_planner_base
from publish_frontiers_node import PublishFrontiersNode


class ExplorationRRTNode:

    def __init__(self):
        self.map_pointcloud_msg = None
        self.odom_msg = None
        self.mi_markers = MarkerArray()
        self.mesh_marker = Marker()
        self.sampled_pc = None

        self.cooldown_time_secs = 40
        self.cooldown_epoch = 0
        self.graph_metric_type = graph_metrics.GraphMetricType.COMBINED

        self.numeric_const_pattern = '[-+]? (?: (?: \d* \. \d+ ) | (?: \d+ \.? ) )(?: [Ee] [+-]? \d+ ) ?'
        self.number_rx = re.compile(self.numeric_const_pattern, re.VERBOSE)

        self.rate_very_slow = rospy.Rate(0.05)
        self.rate_slow = rospy.Rate(1.0)
        self.rate_fast = rospy.Rate(10.0)

        self.mplanner = mesh_planner_node.MeshPlannerNode()
        self.mplanner.init_action_node()

        # publishers
        self.cmd_vel_pub = None
        self.robot_fov_cloud_pub = None

        self.init()
        self.execute_initial_movements()

    def init(self):
        # subscribers
        rospy.Subscriber('/laser_cloud_surround2', PointCloud2, self.map_point_cloud_callback)
        rospy.Subscriber('/integrated_to_init2', Odometry, self.odom_callback)
        rospy.Subscriber('/frontiers_mutual_information', MarkerArray, self.mi_markers_callback)
        rospy.Subscriber('/reconstructed_mesh_marker_normal', Marker, self.mesh_marker_callback)

        # publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.robot_fov_cloud_pub = rospy.Publisher('/robot_fov_cloud', PointCloud2, queue_size=1)
        self.global_target_point = rospy.Publisher('/global_target_path_point', Marker, queue_size=1)
        rospy.sleep(1.)

    def execute_initial_movements(self):
        # move robot front and back to generate first point cloud

        twist_msg = Twist()
        twist_msg.linear.x = 1.0
        for i in range(10):
            self.cmd_vel_pub.publish(twist_msg)
            rospy.sleep(0.1)

        twist_msg.linear.x = -1.0
        for i in range(10):
            self.cmd_vel_pub.publish(twist_msg)
            rospy.sleep(0.1)

        # twist_msg.linear.x = 0.0
        # for i in range(10):
        #     self.cmd_vel_pub.publish(twist_msg)
        #     rospy.sleep(0.1)

        # for i in tqdm(range(3), desc="Sleeping"):
        #     rospy.sleep(1)
            

    def map_point_cloud_callback(self, msg):
        self.map_pointcloud_msg = msg

    def odom_callback(self, msg):
        self.odom_msg = msg

    def mi_markers_callback(self, msg):
        self.mi_markers = msg

    def mesh_marker_callback(self, msg):
        self.mesh_marker = msg

        mesh = o3d.io.read_triangle_mesh(self.mesh_marker.mesh_resource[7:])
        self.sampled_pc = mesh.sample_points_uniformly(number_of_points=len(mesh.vertices))

    @staticmethod
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
            open3d_cloud.points = o3d.Vector3dVector(np.array(xyz))
            open3d_cloud.colors = o3d.Vector3dVector(np.array(rgb) / 255.0)
        else:
            xyz = [(x, y, z) for x, y, z, _ in cloud_data]  # get xyz
            open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))

        return open3d_cloud

    @staticmethod
    def convert_cloud_from_open3d_to_ros(open3d_cloud, frame_id="os1_init"):
        # Convert the datatype of point cloud from Open3D to ROS PointCloud2 (XYZRGB only)

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

        # Set "header"
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id

        # Set "fields" and "cloud_data"
        points = np.asarray(open3d_cloud.points)
        fields=FIELDS_XYZ
        cloud_data=points
    
        # create ros_cloud
        return pc2.create_cloud(header, fields, cloud_data)

    def prepare_pointclouds(self, curr_lidar_pcloud, sampled_pc, src_point=None):
        # LiDAR pointcloud
        pcd = self.convert_cloud_from_ros_to_open3d(curr_lidar_pcloud)
        
        p1_load = np.asarray(pcd.points)
        p2_load = np.asarray(sampled_pc.points)
        pcd.points = o3d.utility.Vector3dVector(np.concatenate((p1_load, p2_load), axis=0))
        downpcd_simple = pcd.voxel_down_sample(voxel_size=0.15)
        downpcd_simple.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=48))
        o3d.io.write_point_cloud("/tmp/test_pointcloud.ply", downpcd_simple, print_progress=True)

        # filter by current point of view
        if src_point:
            print("filter by current point of view...")
            diameter = np.linalg.norm(
                np.asarray(downpcd_simple.get_max_bound()) - np.asarray(downpcd_simple.get_min_bound()))

            camera = [src_point[0], src_point[1], src_point[2] + 0.35]
            radius = diameter * 3

            #print("Remove points that are not visible outside a given view point:", camera)
            #print("Radius:", radius)
            _, pt_map = downpcd_simple.hidden_point_removal(camera, radius)

            view_pcd = downpcd_simple.select_by_index(pt_map)
            #view_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=48))
            view_pcd.paint_uniform_color([0.1, 0.8, 0.8])
            #o3d.visualization.draw_geometries([downpcd_simple, view_pcd])

            downpcd_simple = view_pcd

        np_points = np.asarray(downpcd_simple.points)
        np_normals = np.asarray(downpcd_simple.normals)

        normalized_np_normals = (np_normals / np.linalg.norm(np_normals, axis=1)[:, None])
        clipped_u = np.clip(np.dot(normalized_np_normals, [0, 0, -1]), -1.0, 1.0)
        arccos_u = np.arccos(clipped_u)
        degrees_u = np.degrees(arccos_u)
        degrees_u = np.where(~(degrees_u > 90), degrees_u, np.abs(degrees_u - 180))
    
        inclination_thresh = 33

        np_points_filter = np_points[(degrees_u <= inclination_thresh)]
        np_normals_filter = np_normals[(degrees_u <= inclination_thresh)]

        bad_points = np_points[(degrees_u > inclination_thresh)]

        print("Filtering points/normals by distance..")
        kd_tree = scipy.spatial.cKDTree(np_points_filter)
        filtering_by_dist = [True for _ in np_points_filter]
        for point in bad_points:
            near_idx = kd_tree.query_ball_point(point, r=0.70)
            for idx in near_idx:
                filtering_by_dist[idx] = False

        np_points_filter = np_points_filter[filtering_by_dist]
        np_normals_filter = np_normals_filter[filtering_by_dist]

        filtered_downpcd = o3d.geometry.PointCloud()
        filtered_downpcd.points = o3d.utility.Vector3dVector(np_points_filter)
        filtered_downpcd.normals = o3d.utility.Vector3dVector(np_normals_filter)
        filtered_downpcd.paint_uniform_color([0.1, 0.706, 0])

        bad_pcd = o3d.geometry.PointCloud()
        bad_pcd.points = o3d.utility.Vector3dVector(bad_points)
        bad_pcd.paint_uniform_color([0.99, 0.1, 0])

        if src_point:
            o3d.io.write_point_cloud("/tmp/test_pointcloud_viewpoint_before_outlier.ply", filtered_downpcd, print_progress=True)

            # remove outliers
            #cl, ind = filtered_downpcd.remove_statistical_outlier(nb_neighbors=25, std_ratio=2.0)
            cl, ind = filtered_downpcd.remove_radius_outlier(nb_points=16, radius=1.0)
            filtered_downpcd = filtered_downpcd.select_by_index(ind)

            kd_tree = scipy.spatial.cKDTree(np.asarray(filtered_downpcd.points))
            dist, idx = kd_tree.query([src_point])

            with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
                labels = np.array(filtered_downpcd.cluster_dbscan(eps=2.5, min_points=10, print_progress=True))

            if len(labels) > 0:
                cluster_label = labels[idx[0]]
                indices = [i for i in range(len(labels)) if labels[i] == cluster_label]
                filtered_downpcd = filtered_downpcd.select_by_index(indices)

            o3d.io.write_point_cloud("/tmp/test_pointcloud_viewpoint_after_outlier.ply", filtered_downpcd, print_progress=True)

            ros_pcloud_msg = self.convert_cloud_from_open3d_to_ros(filtered_downpcd)
            self.robot_fov_cloud_pub.publish(ros_pcloud_msg)

        o3d.visualization.draw_geometries([filtered_downpcd])
        return filtered_downpcd

    def publish_and_navigate_path(self, path_msg, wait_for_finish_path=False):

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
            self.mplanner.action_client_done = False
            self.mplanner.action_client.send_goal(goal,
                                                  feedback_cb=self.mplanner.action_client_feedback_callback,
                                                  done_cb=self.mplanner.action_client_done_callback)
            if wait_for_finish_path:
                rospy.loginfo("waiting for action result...")
                while not rospy.is_shutdown() and not self.mplanner.action_client_done:
                    self.rate_slow.sleep()

                action_result = self.mplanner.action_client.get_result()
                rospy.loginfo("action_result:%s", action_result)

        except Exception as e:
            traceback.print_exc()
            rospy.logerr('Error sending action goal %s', e.message)

    def run(self):
        pub_frontiers_obj = PublishFrontiersNode()

        while not rospy.is_shutdown():
            pub_frontiers_obj.run()

            # check if the source and target are valid
            if not self.map_pointcloud_msg or \
                    not self.odom_msg or \
                    not len(self.mi_markers.markers) > 0 or \
                    not self.mesh_marker or \
                    not self.sampled_pc:

                rospy.loginfo("Waiting for data for point cloud RRT...")
                if not self.map_pointcloud_msg:
                    rospy.loginfo("self.map_pointcloud_msg: [%s]", bool(self.map_pointcloud_msg))
                
                if not self.odom_msg:
                    rospy.loginfo("self.odom_msg: [%s]", bool(self.odom_msg))

                if not len(self.mi_markers.markers) > 0:
                    rospy.loginfo("self.mi_markers.markers: [%s]", len(self.mi_markers.markers) > 0)

                if not self.mesh_marker:
                    rospy.loginfo("self.mesh_marker: [%s]", bool(self.mesh_marker))

                if not self.sampled_pc: 
                    rospy.loginfo("self.sampled_pc: [%s]", bool(self.sampled_pc))

                self.rate_slow.sleep()
                continue

            # prepare exploration variables
            curr_lidar_pcloud = self.map_pointcloud_msg
            src_point = (self.odom_msg.pose.pose.position.x,
                         self.odom_msg.pose.pose.position.y,
                         self.odom_msg.pose.pose.position.z)
            curr_mi_markers = self.mi_markers
            #self.mi_markers = MarkerArray()

            rospy.loginfo("Start planning, preparing pointcloud...")
            filtered_downpcd = self.prepare_pointclouds(curr_lidar_pcloud, self.sampled_pc)
            filtered_points = np.asarray(filtered_downpcd.points)
            rospy.loginfo("\tPointcloud prepared, size: %d points", len(filtered_points))

            src_marker = mesh_helper.create_marker(src_point, color=(0.0, 1.0, 0.0), duration=60, marker_id=0)
            self.mplanner.pub_src_point.publish(src_marker)
            self.rate_fast.sleep()

            #rospy.loginfo("Processing filtered points, searching with KDTrees...")
            target_positions = []
            target_faces = []
            target_mi = []
            kd_tree_of_lines = KDTree(np.array(filtered_points), leaf_size=2)

            for mi_marker in curr_mi_markers.markers:
                pos = (mi_marker.pose.position.x, mi_marker.pose.position.y, mi_marker.pose.position.z - 1.0)
                target_positions.append(pos)

                dist, idx = kd_tree_of_lines.query([pos], k=1)
                target_faces.append(idx[0][0])

                numbers = self.number_rx.findall(mi_marker.text)
                if len(numbers) > 0:
                    target_mi.append(float(numbers[0]))
                else:
                    target_mi.append(0.0)
                    rospy.logerr("Mutual information float cannot be extracted from '%s'", mi_marker.text)

            target_mi = np.array(target_mi)
            target_mi = target_mi
            normalized_mi = list(target_mi / target_mi.sum())
            rospy.loginfo("Target faces: %s, Normalized MI: %s", target_faces, normalized_mi)
            
            rospy.loginfo("Initializing RRT planner...")
            rrt_planner = pointcloud_planner_base.PointCloudPlannerBase(filtered_downpcd, [self.graph_metric_type])
            dist, source_face_idx = kd_tree_of_lines.query([src_point], k=1)
            return_dict = rrt_planner.run(source_face_idx[0][0], target_faces, normalized_mi, is_debug=True)
            
            initial_rrt_path = []

            if self.graph_metric_type in return_dict:
                return_dict[graph_metrics.GraphMetricType.GLOBAL] = return_dict[self.graph_metric_type]
                self.mplanner.publish_paths(return_dict)
                path_msg = return_dict[self.graph_metric_type]['path_msg']
                self.publish_and_navigate_path(path_msg)

                # publish global dst point
                initial_rrt_path = [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in path_msg.poses]

                last_path_point = initial_rrt_path[-1]
                last_3d_point = [last_path_point[0], last_path_point[1], last_path_point[2] + 0.6]
                dst_marker = mesh_helper.create_marker(last_3d_point, color=(0.0, 0.0, 1.0), duration=60, marker_id=0)
                self.global_target_point.publish(dst_marker)

                self.rate_fast.sleep()
            else:
                rospy.logerr("No path found in first try! get new target...")
                #self.mi_markers = MarkerArray()
                continue

            selected_frontier = return_dict[self.graph_metric_type]['face_path'][-1]
            selected_frontier_idx = -1
            if selected_frontier in target_faces:
                selected_frontier_idx = target_faces.index(selected_frontier)

            if selected_frontier_idx == -1:
                continue

            # try to go to this specfic frontier avoiding obstacles
            while not rospy.is_shutdown():
                rospy.loginfo("Local path search...")

                src_point = (self.odom_msg.pose.pose.position.x,
                                self.odom_msg.pose.pose.position.y,
                                self.odom_msg.pose.pose.position.z)

                dist = np.linalg.norm(np.array(src_point) - np.array(target_positions[selected_frontier_idx]))
                is_target_reached = dist < 0.45
                rospy.loginfo(("\tIs reached:{} Dist to target:{}".format(is_target_reached, dist)))
                
                if dist < 0.45:
                    print("\tCancelling all goals...")
                    self.mplanner.action_client.cancel_all_goals()
                    break

                if self.mplanner.action_client_done:
                    print("\tTarget reached by action controller")
                    # self.mi_markers = MarkerArray()
                    # remove selected target from MarkerArray
                    del self.mi_markers.markers[selected_frontier_idx]
                    break

                curr_lidar_pcloud = self.map_pointcloud_msg
                filtered_downpcd = self.prepare_pointclouds(curr_lidar_pcloud, self.sampled_pc, src_point)
                filtered_points = np.asarray(filtered_downpcd.points)

                source_face_idx = mesh_helper.find_closer_centroid(filtered_points,
                                                                    src_point,
                                                                    force_return_closer=True)

                initial_len = len(initial_rrt_path)
                print(f"\tinitial_rrt_path len:{initial_len}")
                kd_tree_rrt_path = scipy.spatial.cKDTree(initial_rrt_path)
                sorted_ind = kd_tree_rrt_path.query_ball_point(src_point, r=4, return_sorted=True)
                print("\tsorted_ind:", sorted_ind)
                if len(sorted_ind) > 0:
                    last_ind = sorted_ind[-1]
                    frontier_idx = mesh_helper.find_closer_centroid(filtered_points,
                                                                    initial_rrt_path[last_ind],
                                                                    force_return_closer=True)
                    initial_rrt_path = initial_rrt_path[last_ind:]
                    after_len = len(initial_rrt_path)
                    print(f"\tinitial_rrt_path len after prunning:{after_len}")
                else:
                    frontier_idx = mesh_helper.find_closer_centroid(filtered_points,
                                                                    initial_rrt_path[-1],
                                                                    force_return_closer=True)

                # check if temporal frontier is too close
                # this means that the global frontier is not available
                dist = np.linalg.norm(np.array(src_point) - np.array(filtered_points[frontier_idx]))
                rospy.loginfo(("\tDist to temp frontier:{}".format(dist)))
                if dist < 0.45:
                    print("\tTemporal frontier is too close: Cancelling all goals...")
                    self.mplanner.action_client.cancel_all_goals()
                    self.mi_markers = MarkerArray()
                    break


                print("\tSource:", source_face_idx, "Target:", frontier_idx)

                rrt_planner = pointcloud_planner_base.PointCloudPlannerBase(filtered_downpcd,
                                                                            [self.graph_metric_type])
                return_dict = rrt_planner.run(source_face_idx, [frontier_idx], [1], is_debug=False)
                if self.graph_metric_type in return_dict:
                    self.mplanner.publish_paths(return_dict)
                    path_msg = return_dict[self.graph_metric_type]['path_msg']
                    self.publish_and_navigate_path(path_msg)
                else:
                    rospy.logerr("\tNo path found! get new frontier target")
                    self.mplanner.action_client.cancel_all_goals()
                    self.mi_markers = MarkerArray()
                    break

                rospy.sleep(0.5)
                print("\n\n")

                rospy.loginfo("RRTPlanner slow sleep...")
                print("\n\n")
                self.rate_slow.sleep()


if __name__ == '__main__':
    rospy.init_node('pointcloud_planner_exploration_rrt_main_node')
    rospy.loginfo("RRT exploration node start")

    exp_node = ExplorationRRTNode()
    exp_node.run()

    rospy.loginfo("RRT exploration node stop")

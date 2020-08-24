#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Polygon, PointStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2
import traceback
import pymesh
from recon_surface.srv import MeshFromPointCloud2
from mesh_planner import mesh_planner, mesh_helper

from mesh_planner.graph_metrics import GraphMetric, GraphMetricType
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import sys
import rospkg
import os


class TestMeshPlanner:
    """
    Doc here #todo
    """

    def __init__(self):
        self.map_pointcloud_msg = None
        self.odom_msg = None
        self.clicked_point_msg = None

        # publishers
        self.pub_poly_traj_points = None
        self.pub_path_short = None
        self.pub_path_energy = None
        self.pub_path_traver = None
        self.pub_path_combined = None
        self.pub_path_straight = None
        self.pub_src_point = None
        self.pub_dst_point = None
        self.pub_mesh_marker = None

        self.init_node()

    def init_node(self):
        """
        Define ROS node specifics such as publishers and subscribers
        :return:
        """
        # publishers
        self.pub_path_short = rospy.Publisher('/robot_path_shortest', Path, latch=True, queue_size=1)
        self.pub_path_energy = rospy.Publisher('/robot_path_energy', Path, latch=True, queue_size=1)
        self.pub_path_traver = rospy.Publisher('/robot_path_traversal', Path, latch=True, queue_size=1)
        self.pub_path_straight = rospy.Publisher('/robot_path_straightest', Path, latch=True, queue_size=1)
        self.pub_path_combined = rospy.Publisher('/robot_path_combined', Path, latch=True, queue_size=1)
        self.pub_src_point = rospy.Publisher('/source_path_point', Marker, latch=True, queue_size=1)
        self.pub_dst_point = rospy.Publisher('/target_path_point', Marker, latch=True, queue_size=1)
        self.pub_mesh_marker = rospy.Publisher('/reconstructed_mesh_marker_normal', Marker, latch=True, queue_size=1)

    def publish_paths(self, path_dict, frame_id="/initial_base"):

        for k, v in path_dict.items():
            path = Path()
            path.header.frame_id = frame_id
            path.poses = []

            for point in v:
                pose = PoseStamped()
                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]
                pose.pose.position.z = point[2] + 0.1
                path.poses.append(pose)

            if k == GraphMetricType.SHORTEST:
                self.pub_path_short.publish(path)
            elif k == GraphMetricType.FLATTEST:
                self.pub_path_traver.publish(path)
            elif k == GraphMetricType.ENERGY:
                self.pub_path_energy.publish(path)
            elif k == GraphMetricType.COMBINED:
                self.pub_path_combined.publish(path)
            elif k == GraphMetricType.STRAIGHTEST:
                self.pub_path_straight.publish(path)
            else:
                raise TypeError("No valid Metric Type available for publishing path {}".format(k))


if __name__ == '__main__':
    rospy.init_node('mesh_planner_main_node')

    mplanner = TestMeshPlanner()

    rospy.loginfo("MeshPlanner node start")

    rate_slow = rospy.Rate(1.0)
    rate_fast = rospy.Rate(10.0)

    try:
        rospy.loginfo("Start planning...")
        src = (-5000, -2500, 200)
        dst = (500, -2500, 200)

        src_marker = mesh_helper.create_marker(src, color=(0.0, 1.0, 0.0), duration=0, marker_id=0)
        mplanner.pub_src_point.publish(src_marker)

        dst_marker = mesh_helper.create_marker(dst, color=(0.0, 0.0, 1.0), duration=0, marker_id=1)
        mplanner.pub_dst_point.publish(dst_marker)

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('espeleo_planner')
        mesh_filepath = os.path.join(pkg_path, "test", "map_01_v2.stl")
        rospy.loginfo("mesh_filepath: %s", mesh_filepath)

        mesh_marker = mesh_helper.create_marker((0, 0, 0),
                                               m_scale = 1.0,
                                               color=(0.0, 0.0, 1.0), duration=0, marker_id=2,
                                               mesh_resource="file://" + mesh_filepath)
        mplanner.pub_mesh_marker.publish(mesh_marker)
        rate_fast.sleep()

        # load the mesh and locate the face closer to the src and dst points
        mesh_load = pymesh.load_mesh(mesh_filepath)
        mesh_load.add_attribute("face_centroid")
        centroids = mesh_load.get_face_attribute("face_centroid")

        vertices = mesh_load.vertices
        ver_face = mesh_load.faces

        source_face = mesh_helper.find_face_centroid(centroids, src, force_return_closer=True)

        target_face = mesh_helper.find_face_centroid(centroids, dst, force_return_closer=True)

        # check if src and dst faces are found
        if source_face == -1 or target_face == -1:
            rospy.loginfo("Cannot find the target or source face: src:%d dst:%d", source_face, target_face)
            rate_slow.sleep()
            sys.exit()

        graph_metrics = [GraphMetric(GraphMetricType.SHORTEST, source_face, target_face),
                         GraphMetric(GraphMetricType.FLATTEST, source_face, target_face),
                         GraphMetric(GraphMetricType.ENERGY, source_face, target_face),
                         GraphMetric(GraphMetricType.COMBINED, source_face, target_face),
                         GraphMetric(GraphMetricType.STRAIGHTEST, source_face, target_face)]

        # graph_metrics = [GraphMetric(GraphMetricType.STRAIGHTEST, source_face, target_face)]
        #graph_metrics = [GraphMetric(GraphMetricType.SHORTEST, source_face, target_face)]

        planner = mesh_planner.MeshPathFinder(mesh_filepath, graph_metrics)
        return_dict = planner.run()
        mplanner.publish_paths(return_dict)
        #print "return_dict:", return_dict
        rospy.signal_shutdown(0)
        sys.exit()
    except Exception as e:
        tb = traceback.format_exc()
        rospy.logerr("Main Exception: %s", str(tb))

    rospy.loginfo("MeshPlanner node stop")

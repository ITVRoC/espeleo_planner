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


class MeshPlanner:
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

        self.init_node()

    def init_node(self):
        """
        Define ROS node specifics such as publishers and subscribers
        :return:
        """
        # publishers
        self.pub_poly_traj_points = rospy.Publisher('/espeleo/traj_points_polygon', Polygon, latch=True, queue_size=1)
        self.pub_path_short = rospy.Publisher('/robot_path_shortest', Path, latch=True, queue_size=1)
        self.pub_path_energy = rospy.Publisher('/robot_path_energy', Path, latch=True, queue_size=1)
        self.pub_path_traver = rospy.Publisher('/robot_path_traversal', Path, latch=True, queue_size=1)
        self.pub_path_straight = rospy.Publisher('/robot_path_straightest', Path, latch=True, queue_size=1)
        self.pub_path_combined = rospy.Publisher('/robot_path_combined', Path, latch=True, queue_size=1)
        self.pub_src_point = rospy.Publisher('/source_path_point', Marker, latch=True, queue_size=1)
        self.pub_dst_point = rospy.Publisher('/target_path_point', Marker, latch=True, queue_size=1)

        # subscribers
        rospy.Subscriber('/octomap_point_cloud_centers', PointCloud2, self.map_point_cloud_callback)
        rospy.Subscriber('/integrated_to_init2', Odometry, self.odom_callback)
        rospy.Subscriber('/clicked_point', PointStamped, self.clicked_point_callback)

    def odom_callback(self, msg):
        self.odom_msg = msg

    def map_point_cloud_callback(self, msg):
        self.map_pointcloud_msg = msg

    def clicked_point_callback(self, msg):
        self.clicked_point_msg = msg

    def reset_data(self):
        self.odom_msg = None
        self.clicked_point_msg = None
        self.map_pointcloud_msg = None

    def is_ready_to_plan(self):
        if self.odom_msg is not None and \
            self.clicked_point_msg is not None and \
                self.map_pointcloud_msg is not None:
            return True
        else:
            return False

    def get_plan_data(self):
        return self.odom_msg, self.clicked_point_msg, self.map_pointcloud_msg

    def publish_paths(self, path_dict, frame_id="/initial_base"):

        for k, v in path_dict.items():
            path = Path()
            path.header.frame_id = frame_id
            path.poses = []

            for point in v:
                pose = PoseStamped()
                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]
                pose.pose.position.z = point[2]
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

    mplanner = MeshPlanner()

    rospy.loginfo("MeshPlanner node start")

    rate_slow = rospy.Rate(1.0)
    rate_fast = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            # check if the source and target are valid
            if not mplanner.is_ready_to_plan():
                rospy.loginfo("Planner waiting for data...")
                rate_slow.sleep()
                continue

            rospy.loginfo("Start planning...")

            src, dst, pcloud = mplanner.get_plan_data()
            mplanner.reset_data()

            src_marker = mesh_helper.create_marker((src.pose.pose.position.x,
                                                    src.pose.pose.position.y,
                                                    src.pose.pose.position.z),
                                                   color=(0.0, 1.0, 0.0), duration=30, marker_id=0)
            mplanner.pub_src_point.publish(src_marker)

            dst_marker = mesh_helper.create_marker((dst.point.x,
                                                    dst.point.y,
                                                    dst.point.z),
                                                   color=(0.0, 0.0, 1.0), duration=30, marker_id=1)
            mplanner.pub_dst_point.publish(dst_marker)
            rate_fast.sleep()  # allow publishing the markers without waiting to the last rate.sleep()

            # run the service to convert the point cloud to mesh
            mesh_filepath = None
            try:
                rospy.wait_for_service('/mesh_from_pointclouds', timeout=3)
                mesh_from_pointcloud = rospy.ServiceProxy('/mesh_from_pointclouds', MeshFromPointCloud2)
                resp1 = mesh_from_pointcloud(pcloud)
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

            # load the mesh and locate the face closer to the src and dst points
            mesh_load = pymesh.load_mesh(mesh_filepath)
            mesh_load.add_attribute("face_centroid")
            centroids = mesh_load.get_face_attribute("face_centroid")

            vertices = mesh_load.vertices
            ver_face = mesh_load.faces

            source_face = mesh_helper.find_closer_centroid(centroids,
                                                          (src.pose.pose.position.x,
                                                           src.pose.pose.position.y,
                                                           src.pose.pose.position.z),
                                                          force_return_closer=True)

            target_face = mesh_helper.find_closer_centroid(centroids,
                                                          (dst.point.x,
                                                           dst.point.y,
                                                           dst.point.z),
                                                          force_return_closer=True)

            # check if src and dst faces are found
            if source_face == -1 or target_face == -1:
                rospy.loginfo("Cannot find the target or source face: src:%d dst:%d", source_face, target_face)
                rate_slow.sleep()
                continue

            # graph_metrics = [GraphMetric(GraphMetricType.SHORTEST, source_face, target_face),
            #                  GraphMetric(GraphMetricType.FLATTEST, source_face, target_face),
            #                  GraphMetric(GraphMetricType.ENERGY, source_face, target_face),
            #                  GraphMetric(GraphMetricType.COMBINED, source_face, target_face),
            #                  GraphMetric(GraphMetricType.STRAIGHTEST, source_face, target_face)]

            #graph_metrics = [GraphMetric(GraphMetricType.SHORTEST, source_face, target_face)]
            #graph_metrics = [GraphMetric(GraphMetricType.FLATTEST, source_face, target_face)]
            #graph_metrics = [GraphMetric(GraphMetricType.ENERGY, source_face, target_face)]
            graph_metrics = [GraphMetric(GraphMetricType.COMBINED, source_face, target_face)]
            #graph_metrics = [GraphMetric(GraphMetricType.STRAIGHTEST, source_face, target_face)]

            planner = mesh_planner.MeshPathFinder(mesh_filepath, graph_metrics)
            return_dict = planner.run()
            mplanner.publish_paths(return_dict)
            print "return_dict:", return_dict
        except Exception as e:
            tb = traceback.format_exc()
            rospy.logerr("Main Exception: %s", str(tb))

        rate_slow.sleep()

    rospy.loginfo("MeshPlanner node stop")

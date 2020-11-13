#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import Polygon, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2
from graph_metrics import GraphMetricType
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
import espeleo_control.msg
from geometry_msgs.msg import PoseStamped


class MeshPlannerNode:
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
        self.pub_frontiers_ground_pts = None
        self.pub_frontiers_ground_centroids = None
        self.pub_frontiers_ground_centroids_labels = None
        self.pub_frontiers_ground_trav_labels = None

        self.action_client = None
        self.action_client_feedback = 0
        self.action_client_result = None
        self.action_client_done = False

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
        self.pub_frontiers_ground_pts = rospy.Publisher('/frontiers_ground_pts', MarkerArray, latch=True,
                                                        queue_size=1)
        self.pub_frontiers_ground_centroids = rospy.Publisher('/frontiers_ground_centroids', MarkerArray, latch=True,
                                                              queue_size=1)
        self.pub_frontiers_ground_centroids_labels = rospy.Publisher('/frontiers_ground_centroids_labels', MarkerArray,
                                                                     latch=True, queue_size=1)
        self.pub_frontiers_ground_trav_labels = rospy.Publisher('/frontiers_ground_centroids_traversability_labels', MarkerArray,
                                                                     latch=True, queue_size=1)

        # subscribers
        rospy.Subscriber('/laser_cloud_surround2', PointCloud2, self.map_point_cloud_callback)
        rospy.Subscriber('/integrated_to_init2', Odometry, self.odom_callback)
        
    def init_manual_node(self):
        rospy.Subscriber('/clicked_point', PointStamped, self.clicked_point_callback)
        
    def init_action_node(self):
        self.action_client = actionlib.SimpleActionClient('espeleo_control_action',
                                                          espeleo_control.msg.NavigatePathAction)
        self.action_client.wait_for_server(rospy.Duration.from_sec(3))

    def action_client_feedback_callback(self, feedback):
        self.action_client_feedback = feedback

    def action_client_done_callback(self, state, result):
        rospy.loginfo("action_client_done_callback")
        rospy.loginfo("state:%s result:%s", str(state), str(result))

        self.action_client_result = result
        self.action_client_done = True
        self.action_client_feedback = 0

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

    def is_ready_to_explore(self):
        if self.odom_msg is not None and \
                self.map_pointcloud_msg is not None:
            return True
        else:
            return False

    def get_plan_data(self):
        return self.odom_msg, self.clicked_point_msg, self.map_pointcloud_msg

    def publish_paths(self, path_dict, frame_id="/os1_init"):

        for k, v in path_dict.items():
            path = Path()
            path.header.frame_id = frame_id
            path.poses = []

            for point in v["path"]:
                pose = PoseStamped()
                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]
                pose.pose.position.z = point[2]
                path.poses.append(pose)

            path_dict[k]["path_msg"] = path

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

        return path_dict

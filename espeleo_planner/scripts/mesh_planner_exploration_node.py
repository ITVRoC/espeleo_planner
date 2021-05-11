#!/usr/bin/env python

import rospy
import traceback
import pymesh
from recon_surface.srv import MeshFromPointCloud2
from visualization_msgs.msg import MarkerArray
import random
import numpy as np

from mesh_planner.graph_metrics import GraphMetricType
import espeleo_control.msg
import std_msgs.msg
from geometry_msgs.msg import Polygon, Point32, Point
from mesh_planner import mesh_helper, graph_metrics, mesh_planner_base, mesh_planner_node
from espeleo_planner.srv import processAllFrontiers, processAllFrontiersUsingSTLOctomap
from geometry_msgs.msg import Twist

from pyquaternion import Quaternion


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


def check_is_dest_reachable():
    return True


if __name__ == '__main__':
    rospy.init_node('mesh_planner_exploration_main_node')

    mplanner = mesh_planner_node.MeshPlannerNode()
    mplanner.init_action_node()

    rospy.loginfo("MeshPlanner node start")

    rate_very_slow = rospy.Rate(0.05)
    rate_slow = rospy.Rate(1.0)
    rate_fast = rospy.Rate(10.0)

    cmd_vel_pub = rospy.Publisher('/origin_gt_point', Point32, queue_size=1)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.sleep(3.)

    twist_msg = Twist()
    twist_msg.linear.x = 1.0
    for i in xrange(30):
        cmd_vel_pub.publish(twist_msg)
        #rate_fast.sleep()
        rospy.sleep(0.2)

    twist_msg.linear.x = -1.0
    for i in xrange(30):
        cmd_vel_pub.publish(twist_msg)
        #rate_fast.sleep()
        rospy.sleep(0.2)

    twist_msg.linear.x = 0.0
    for i in xrange(30):
        cmd_vel_pub.publish(twist_msg)
        #rate_fast.sleep()
        rospy.sleep(0.2)

    error_counter = 0

    # move robot front and back to generate first point cloud
    while not rospy.is_shutdown():
        try:
            # check if the source and target are valid
            if not mplanner.is_ready_to_explore():
                rospy.loginfo("Planner waiting for data...")
                rate_slow.sleep()
                continue

            if error_counter > 5:
                rospy.logerr("error_counter > 5, stopping exploration...")
                rate_slow.sleep()
                break

            rospy.loginfo("Start planning...")

            src, _, pcloud = mplanner.get_plan_data()
            mplanner.reset_data()

            src_marker = mesh_helper.create_marker((src.pose.pose.position.x,
                                                    src.pose.pose.position.y,
                                                    src.pose.pose.position.z),
                                                   color=(0.0, 1.0, 0.0), duration=60, marker_id=0)
            mplanner.pub_src_point.publish(src_marker)
            rate_fast.sleep()  # allow publishing the markers without waiting to the last rate.sleep()

            # run the service to convert the point cloud to mesh
            mesh_filepath = None
            try:
                rospy.wait_for_service('/mesh_from_pointclouds', timeout=3)
                mesh_from_pointcloud = rospy.ServiceProxy('/mesh_from_pointclouds', MeshFromPointCloud2)
                mesh_src_point = Point(0.0, 0.0, 0.0)
                resp1 = mesh_from_pointcloud(pcloud, mesh_src_point)
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
            #mesh_load, info = pymesh.split_long_edges(mesh_load, 0.3)
            mesh_load.add_attribute("face_centroid")
            mesh_load.add_attribute("face_normal")
            centroids = mesh_load.get_face_attribute("face_centroid")
            normals = mesh_load.get_face_attribute("face_normal")

            vertices = mesh_load.vertices
            ver_face = mesh_load.faces

            source_face = mesh_helper.find_closer_centroid(centroids,
                                                           (src.pose.pose.position.x,
                                                            src.pose.pose.position.y,
                                                            src.pose.pose.position.z),
                                                           force_return_closer=True)

            # check if src and dst faces are found
            if source_face == -1:
                rospy.loginfo("Cannot find the source face: src:%d", source_face)
                rate_slow.sleep()
                continue

            # graph_metric_types = [graph_metrics.GraphMetricType.SHORTEST,
            #                  graph_metrics.GraphMetricType.FLATTEST,
            #                  graph_metrics.GraphMetricType.ENERGY,
            #                  graph_metrics.GraphMetricType.COMBINED,
            #                  graph_metrics.GraphMetricType.STRAIGHTEST]

            # graph_metric_types = [graph_metrics.GraphMetricType.STRAIGHTEST]
            #graph_metric_types = [graph_metrics.GraphMetricType.SHORTEST]
            graph_metric_types = [graph_metrics.GraphMetricType.COMBINED]

            planner = mesh_planner_base.MeshPlannerBase(mesh_filepath, graph_metric_types)
            G = planner.create_graph_from_mesh()
            G, f_visit_ids, f_centroids_ids, filtered_reachable_frontiers_ids = planner.prepare_graph(G, source_face)

            rospy.loginfo("Estimated frontiers: %f", len(f_visit_ids))

            # publish frontiers points
            frontier_pts_arr = MarkerArray()
            for f_id in filtered_reachable_frontiers_ids:
                x, y, z = centroids[f_id]
                f_marker = mesh_helper.create_marker((x,
                                                      y,
                                                      z),
                                                     color=(0.7, 0.0, 0.0), duration=60, m_scale=0.5, marker_id=f_id)
                frontier_pts_arr.markers.append(f_marker)
            mplanner.pub_frontiers_ground_pts.publish(frontier_pts_arr)

            # publish frontier centroids
            frontier_centroids_arr = MarkerArray()
            frontier_centroids_arr_labels = MarkerArray()
            for i, f_id in enumerate(f_centroids_ids):
                x, y, z = centroids[f_id]
                R = rotation_matrix_from_vectors((0, 0, 1), normals[f_id])
                q = Quaternion(matrix=R)

                f_marker = mesh_helper.create_marker((x,
                                                      y,
                                                      z),
                                                     orientation=[q[3], q[0], q[1], q[2]],
                                                     color=(0.7, 0.0, 0.0), duration=60, m_scale=1.0, marker_id=f_id,
                                                     marker_type=1)
                frontier_centroids_arr.markers.append(f_marker)

                f_label_marker = mesh_helper.create_marker((x,
                                                      y,
                                                      z + 4.0),
                                                     color=(0.7, 0.0, 0.0), duration=60, m_scale=0.8, marker_id=i,
                                                     marker_type=9, marker_text="Frontier {}".format(i))
                frontier_centroids_arr_labels.markers.append(f_label_marker)

            mplanner.pub_frontiers_ground_centroids.publish(frontier_centroids_arr)
            mplanner.pub_frontiers_ground_centroids_labels.publish(frontier_centroids_arr_labels)

            # plot 3D graph
            # planner.plot_graph_3d(G,
            #                       title="Frontier test",
            #                       source_id=source_face,
            #                       reachable_frontiers_ids=list(filtered_reachable_frontiers_ids),
            #                       frontier_centroids_ids=f_centroids_ids)

            # run the service to convert the point cloud to mesh
            frontiers_mi = []
            # NORMAL OCTOMAP
            # try:
            #     rospy.wait_for_service('/process_all_frontiers', timeout=3)
            #     frontier_mi_service = rospy.ServiceProxy('/process_all_frontiers', processAllFrontiers)
            #     resp1 = frontier_mi_service()
            #     frontiers_mi = resp1.mi_data
            #     rospy.loginfo("process_all_frontiers result: %s %s", resp1, frontiers_mi)
            # except rospy.ServiceException as e:
            #     rospy.logerr("Service call failed: %s", e)
            # except Exception as e:
            #     rospy.logerr("Exception: %s", e)

            # STL OCTOMAP
            try:
                rospy.wait_for_service('/process_all_frontiers_stl', timeout=3)
                frontier_mi_service = rospy.ServiceProxy('/process_all_frontiers_stl', processAllFrontiersUsingSTLOctomap)
                resp1 = frontier_mi_service(mesh_filepath)
                frontiers_mi = resp1.mi_data
                rospy.loginfo("process_all_frontiers result: %s %s", resp1, frontiers_mi)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s", e)
            except Exception as e:
                rospy.logerr("Exception: %s", e)

            if frontiers_mi is None or len(frontiers_mi) <= 0:
                error_counter += 1
                rospy.logerr("process_all_frontiers is None, cannot continue with the planning")
                rate_slow.sleep()
                continue
            error_counter = 0

            results = []
            frontier_arr_trav_labels = MarkerArray()
            for i, n_id in enumerate(f_visit_ids):
                return_dict = planner.run(source_face, n_id)

                trav_cost = return_dict[graph_metric_types[0]]['cost']
                total_cost = frontiers_mi[i]/trav_cost

                return_dict[graph_metric_types[0]]['total_cost'] = total_cost

                #print "return_dict:", return_dict
                rospy.loginfo("%s) return_dict: %s", n_id, trav_cost)
                results.append(return_dict[graph_metric_types[0]])

                x, y, z = centroids[f_centroids_ids[i]]
                f_label_marker = mesh_helper.create_marker((x,
                                                            y,
                                                            z + 2.5),
                                                           color=(1.0, 1.0, 1.0), duration=60, m_scale=0.8, marker_id=i,
                                                           marker_type=9,
                                                           marker_text="c:{:.3f} mi/c:{:.3f}".format(
                                                               trav_cost, total_cost))
                frontier_arr_trav_labels.markers.append(f_label_marker)
            mplanner.pub_frontiers_ground_trav_labels.publish(frontier_arr_trav_labels)

            print "results:", results
            if len(results) <= 0:
                rospy.logerr("There are no more frontiers to explore, fallback behaviour?")
                continue

            # ------------------------------- #
            # to use for the minimum distance #
            # ------------------------------- #
            # min_cost_item = min(results, key=lambda x: x['cost'])
            # print "min_cost_item:", min_cost_item
            # path_dict_msg = mplanner.publish_paths({graph_metrics.GraphMetricType.COMBINED: min_cost_item})

            # ---------------------------------------------------------- #
            # to use to the greatest proportion mutual_info/traverability
            # ---------------------------------------------------------- #
            max_cost_item = max(results, key=lambda x: x['total_cost'])
            print "max_cost_item:", max_cost_item
            path_dict_msg = mplanner.publish_paths({graph_metrics.GraphMetricType.COMBINED: max_cost_item})

            # ----------------------- #
            # to use random selection
            # ----------------------- #
            # path_dict_msg = mplanner.publish_paths({graph_metrics.GraphMetricType.COMBINED: random.choice(results)})

            #print "path_dict_msg:", path_dict_msg
            path_msg = path_dict_msg[graph_metrics.GraphMetricType.COMBINED]['path_msg']

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

                rospy.loginfo("waiting for action result...")
                while not rospy.is_shutdown() and not mplanner.action_client_done:
                    is_reachable = check_is_dest_reachable()
                    if not is_reachable:
                        mplanner.action_client.cancel_all_goals()
                        break

                    rate_slow.sleep()

                #mplanner.action_client.wait_for_result()
                action_result = mplanner.action_client.get_result()
                rospy.loginfo("action_result:%s", action_result)
            except Exception as e:
                traceback.print_exc()
                rospy.logerr('Error sending action goal %s', e.message)
            # rospy.logwarn("CONTINUE CONTINUE CONTINUE debug DEBUG DEBUG")

            rospy.loginfo("MeshPlanner very slow sleep......")
            rate_very_slow.sleep()
        except Exception as e:
            tb = traceback.format_exc()
            rospy.logerr("Main Exception: %s", str(tb))

        rate_slow.sleep()

    rospy.loginfo("MeshPlanner node stop")

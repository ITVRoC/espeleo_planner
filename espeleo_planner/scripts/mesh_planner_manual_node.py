#!/usr/bin/env python

import rospy
import traceback
import pymesh
from recon_surface.srv import MeshFromPointCloud2
from mesh_planner import mesh_helper, graph_metrics, mesh_planner_base, mesh_planner_node


if __name__ == '__main__':
    rospy.init_node('mesh_planner_manual_main_node')

    mplanner = mesh_planner_node.MeshPlannerNode()
    mplanner.init_manual_node()

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

            # graph_metric_types = [graph_metrics.GraphMetricType.SHORTEST,
            #                  graph_metrics.GraphMetricType.FLATTEST,
            #                  graph_metrics.GraphMetricType.ENERGY,
            #                  graph_metrics.GraphMetricType.COMBINED,
            #                  graph_metrics.GraphMetricType.STRAIGHTEST]

            # graph_metric_types = [graph_metrics.GraphMetricType.STRAIGHTEST]
            graph_metric_types = [graph_metrics.GraphMetricType.SHORTEST]

            planner = mesh_planner_base.MeshPlannerBase(mesh_filepath, graph_metric_types)
            return_dict = planner.run(source_face, target_face)
            mplanner.publish_paths(return_dict)
            print "return_dict:", return_dict
        except Exception as e:
            tb = traceback.format_exc()
            rospy.logerr("Main Exception: %s", str(tb))

        rate_slow.sleep()

    rospy.loginfo("MeshPlanner node stop")

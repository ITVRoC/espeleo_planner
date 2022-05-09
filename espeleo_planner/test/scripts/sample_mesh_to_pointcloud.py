#!/usr/bin/env python

import sys
import os
import rospkg
import rospy
import open3d as o3d

rospack = rospkg.RosPack()
package_path = rospack.get_path('espeleo_planner')
sys.path.append(os.path.join(package_path, "test"))
sys.path.append(os.path.join(package_path, "scripts"))

if __name__ == '__main__':

    mesh = o3d.io.read_triangle_mesh("/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/test/maps/map_01_frontiers.stl")

    mesh_vertices_size = len(mesh.vertices)
    sample_num = mesh_vertices_size
    print("sample_num:", sample_num)

    sampled_pc = mesh.sample_points_uniformly(number_of_points=sample_num)

    o3d.visualization.draw_geometries([sampled_pc])

    # pcd = o3d.geometry.PointCloud()
    # pcd.points = mesh.vertices
    # pcd.colors = mesh.vertex_colors
    # pcd.normals = mesh.vertex_normals
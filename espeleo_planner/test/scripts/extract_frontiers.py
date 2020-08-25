#!/usr/bin/env python

import rospy
import pymesh
import rospkg
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


def extract_frontier_from_mesh(mesh_filepath):
    filename = os.path.splitext(os.path.basename(mesh_filepath))[0]
    mesh = pymesh.load_mesh(mesh_filepath)

    mesh.enable_connectivity()  # enables connectivity on mesh
    mesh.add_attribute("face_centroid")  # adds the face centroids to be accessed
    mesh.add_attribute("face_normal")  # adds the face normals to be accessed
    mesh.add_attribute("vertex_valance")

    faces = mesh.faces
    centroids = mesh.get_face_attribute("face_centroid")
    normals = mesh.get_face_attribute("face_normal")
    vertex_valance = mesh.get_vertex_attribute("vertex_valance")

    #print vertex_valance
    frontiers = set()

    for face_id in range(0, mesh.num_faces):
        adj_faces = mesh.get_face_adjacent_faces(face_id)
        if len(adj_faces) <= 2:
            #print centroids[face_id]
            p = centroids[face_id]
            p = (p[0], p[1], p[2])
            frontiers.add(p)

    pts, _ = pymesh.mesh_to_graph(mesh)
    x, y, z = zip(*pts)
    x = np.array(x)
    y = np.array(y)
    z = np.array(z)
    fig = plt.figure()  # figsize=(800 / 72, 800 / 72)
    ax = plt.axes(projection='3d')
    ax.set_title(filename)
    # ax.scatter3D(x, y, z, c=z, cmap='Greens');
    ax.scatter3D(x, y, z, s=[1.0 for n in range(len(x))], c="blue")
    set_axes_equal(ax)

    # print(frontiers)

    x, y, z = zip(*frontiers)
    x = np.array(x)
    y = np.array(y)
    z = np.array(z)
    ax.scatter3D(x, y, z, s=[3.0 for n in range(len(x))], c="red")

    plt.show()


if __name__ == '__main__':
    rospy.init_node('filter_mesh_node')
    rospy.loginfo("filter_mesh_node start")

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('espeleo_planner')

    test_files = [
        {"map": "map_01_frontiers.stl",
         "pos": (-4, 0, 0)},
        {"map": "map_02_stairs_cavelike.stl",
         "pos": (-4, -1.25, 0.5)},
        {"map": "map_03_narrow_passage.stl",
         "pos": (-4, -1.25, 0.5)},
        {"map": "map_03_narrow_passage_v2.stl",
         "pos": (-4, -1.25, 0.5)},
        {"map": "map_04_stairs_perfect.stl",
         "pos": (-4, -1.25, 0.5)},
        {"map": "map_05_cavelike.stl",
         "pos": (0, 0, 0)},
    ]

    for test in test_files[:]:
        mesh_path = os.path.join(package_path, "test", "maps", test["map"])
        extract_frontier_from_mesh(mesh_path)








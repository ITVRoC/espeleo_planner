#!/usr/bin/env python

import rospy
import pymesh
import rospkg
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

if __name__ == '__main__':
    rospy.init_node('filter_mesh_node')
    rospy.loginfo("filter_mesh_node start")

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('espeleo_planner')

    mesh_path = os.path.join(package_path, "test", "map_frontiers.stl")
    mesh = pymesh.load_mesh(mesh_path)

    mesh.enable_connectivity()  # enables connectivity on mesh
    mesh.add_attribute("face_centroid")  # adds the face centroids to be accessed
    mesh.add_attribute("face_normal")  # adds the face normals to be accessed
    mesh.add_attribute("vertex_valance")

    faces = mesh.faces
    centroids = mesh.get_face_attribute("face_centroid")
    normals = mesh.get_face_attribute("face_normal")
    vertex_valance = mesh.get_vertex_attribute("vertex_valance")

    print vertex_valance
    frontiers = set()

    for face_id in range(0, mesh.num_faces):
        adj_faces = mesh.get_face_adjacent_faces(face_id)
        if len(adj_faces) <= 2:
            print centroids[face_id]
            p = centroids[face_id]
            p = (p[0], p[1], p[2])
            frontiers.add(p)

    pts, _ = pymesh.mesh_to_graph(mesh)
    x, y, z = zip(*pts)
    x = np.array(x)
    y = np.array(y)
    z = np.array(z)
    fig = plt.figure() # figsize=(800 / 72, 800 / 72)
    ax = plt.axes(projection='3d')
    #ax.scatter3D(x, y, z, c=z, cmap='Greens');
    ax.scatter3D(x, y, z, s=[1.0 for n in range(len(x))], c="blue")

    #print(frontiers)

    x, y, z = zip(*frontiers)
    x = np.array(x)
    y = np.array(y)
    z = np.array(z)
    ax.scatter3D(x, y, z, s=[3.0 for n in range(len(x))], c="red")

    plt.show()


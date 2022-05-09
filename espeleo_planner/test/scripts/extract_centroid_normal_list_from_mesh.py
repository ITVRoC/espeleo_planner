#!/usr/bin/env python

import rospy
import pymesh
import rospkg
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def extract_centroid_normal_list(mesh_filepath):
    mesh = pymesh.load_mesh(mesh_filepath)

    mesh.enable_connectivity()  # enables connectivity on mesh
    mesh.add_attribute("face_centroid")  # adds the face centroids to be accessed
    mesh.add_attribute("face_normal")  # adds the face normals to be accessed
    mesh.add_attribute("vertex_valance")

    faces = mesh.faces
    centroids = mesh.get_face_attribute("face_centroid")
    normals = mesh.get_face_attribute("face_normal")
    vertex_valance = mesh.get_vertex_attribute("vertex_valance")

    with open('/tmp/centroid_face_normals.txt', 'w') as file:
        file.write("x\ty\tz\tnx\tny\tnz\n")
        for face_id in range(0, mesh.num_faces):
            p = centroids[face_id]
            n = normals[face_id]

            file.write("{}\t{}\t{}\t{}\t{}\t{}\n".format(p[0], p[1], p[2], n[0], n[1], n[2]))


if __name__ == '__main__':
    rospy.init_node('filter_mesh_node')
    rospy.loginfo("filter_mesh_node start")

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('espeleo_planner')

    test_files = [
        {"map": "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/recon_surface/tmp_reconstructions/25-02-2021_16:40:21_mesh.stl"}
    ]

    for test in test_files[:]:
        extract_centroid_normal_list(test["map"])








#!/usr/bin/env python

import sys
import os
import rospkg
import rospy
import pymesh
import networkx as nx
import numpy as np
from mayavi import mlab
from scipy import spatial

rospack = rospkg.RosPack()
package_path = rospack.get_path('espeleo_planner')
scripts_path = os.path.join(package_path, "scripts")
sys.path.append(scripts_path)

import mesh_planner
from mesh_planner import graph_search


def create_graph(mesh, centroids, normals, robot_pos,
                 traversal_tresh=35):

    print("Creating Graph... num faces:", mesh.num_faces)

    G = nx.Graph()

    for face_idx in xrange(mesh.num_faces):
        face_inclination = graph_search.MeshGraphSearch.calculate_traversal_angle(normals[face_idx])
        if traversal_tresh < face_inclination < 180 - traversal_tresh:
            continue

        G.add_node(face_idx)

    for face_idx in list(G.nodes()):
        face_vertexes = mesh.faces[face_idx]
        for v in face_vertexes:
            vertex_adj_faces = mesh.get_vertex_adjacent_faces(v)
            for face_adjacent in vertex_adj_faces:
                if face_adjacent != face_idx and G.has_node(face_adjacent):
                    G.add_edge(face_idx, face_adjacent, weight=1)

    # print "G node_list:", len(list(G.nodes())), sorted(list(G.nodes()))
    # print "G edge_list:", len(list(G.edges())), sorted(list(G.edges()))

    g_centroids = [(centroids[v][0], centroids[v][1], centroids[v][2]) for v in sorted(G.nodes())]
    centroid_g_dict = {i: v for i, v in enumerate(sorted(G.nodes()))}

    closer_centroid_idx = mesh_planner.mesh_helper.find_closer_centroid(g_centroids, robot_pos,
                                                                        force_return_closer=True)
    conn_nodes = nx.node_connected_component(G, centroid_g_dict[closer_centroid_idx])
    Gconn = G.subgraph(conn_nodes).copy()
    Gconn_original = Gconn.copy()

    # estimate borders of the remainder graph
    # border_nodes = [Gconn.info(v) for v in sorted(Gconn.nodes())]
    border_centroids = []
    for v in sorted(Gconn.nodes()):
        if nx.degree(G, v) <= 9:
            border_centroids.append((centroids[v][0], centroids[v][1], centroids[v][2]))

    # remove nodes from graph that are near to the borders
    # given a distance treshold
    border_kdtree = spatial.KDTree(border_centroids)
    border_tresh = 0.4
    for v in list(Gconn.nodes()):
        point = centroids[v]
        distances, nearest_idx = border_kdtree.query([point])
        obstacle_d = distances[0]
        if obstacle_d <= border_tresh:
            Gconn.remove_node(v)

    # remove small connected components
    for component in list(nx.connected_components(G)):
        if len(component) < 3:
            for node in component:
                G.remove_node(node)

    f1 = mlab.figure("Borderless graph", bgcolor=(0, 0, 0))
    mlab.clf()
    plot_points(Gconn, centroids, border_centroids)

    f2 = mlab.figure("Original graph", bgcolor=(0, 0, 0))
    mlab.clf()
    plot_points(Gconn_original, centroids, border_centroids)

    mlab.sync_camera(f1, f2)
    mlab.show()


def plot_points(Gconn, centroids, border_centroids):
    gcon_centroids = [(centroids[v][0], centroids[v][1], centroids[v][2]) for v in sorted(Gconn.nodes())]
    xyz = np.array(gcon_centroids)
    scalars = xyz[:, 2]  # np.array(list(Gconn.nodes())) #xyz[:, 2]  #+ 5

    pts = mlab.points3d(xyz[:, 0], xyz[:, 1], xyz[:, 2],
                        scalars,
                        scale_factor=0.1,
                        scale_mode='none',
                        colormap='Blues',
                        resolution=20)

    centroid_gcon_dict = {v: int(i) for i, v in enumerate(gcon_centroids)}
    edge_list = []
    for e in Gconn.edges():
        e1 = (centroids[e[0]][0], centroids[e[0]][1], centroids[e[0]][2])
        e2 = (centroids[e[1]][0], centroids[e[1]][1], centroids[e[1]][2])
        edge_list.append([centroid_gcon_dict[e1], centroid_gcon_dict[e2]])

    edge_list = np.array(edge_list)
    pts.mlab_source.dataset.lines = np.array(edge_list)
    lines = mlab.pipeline.stripper(pts)
    mlab.pipeline.surface(lines, color=(0.2, 0.4, 0.5), line_width=1, opacity=.4)

    xyz_borders = np.array(border_centroids)
    scalars_d2 = np.ones(xyz_borders.shape[0])
    mlab.points3d(xyz_borders[:, 0], xyz_borders[:, 1], xyz_borders[:, 2],
                         scalars_d2,
                         scale_factor=0.2,
                         scale_mode='none',
                         color=(1.0, 0.0, 0.0),
                         resolution=20)


if __name__ == '__main__':
    rospy.init_node('expand_borders_node')
    rospy.loginfo("expand_borders_node start")

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
        robot_pos = test["pos"]

        mesh = pymesh.load_mesh(mesh_path)

        mesh.enable_connectivity()  # enables connectivity on mesh
        mesh.add_attribute("face_centroid")  # adds the face centroids to be accessed
        mesh.add_attribute("face_normal")  # adds the face normals to be accessed
        mesh.add_attribute("vertex_valance")

        faces = mesh.faces
        centroids = mesh.get_face_attribute("face_centroid")
        normals = mesh.get_face_attribute("face_normal")
        vertex_valance = mesh.get_vertex_attribute("vertex_valance")

        create_graph(mesh, centroids, normals, robot_pos)


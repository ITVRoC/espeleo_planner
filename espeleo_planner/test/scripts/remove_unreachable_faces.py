#!/usr/bin/env python

import sys
import os
import math
import rospkg
import rospy
import pymesh
import networkx as nx
import numpy as np
from scipy import spatial
import matplotlib.pyplot as plt
from mayavi import mlab
from sklearn.cluster import DBSCAN

rospack = rospkg.RosPack()
package_path = rospack.get_path('espeleo_planner')
scripts_path = os.path.join(package_path, "scripts")
sys.path.append(scripts_path)

import mesh_planner
from mesh_planner import graph_search

# remove faces
# https://github.com/PyMesh/PyMesh/issues/118

def get_centroid_of_pts(arr):
    length = arr.shape[0]
    sum_x = np.sum(arr[:, 0])
    sum_y = np.sum(arr[:, 1])
    sum_z = np.sum(arr[:, 2])
    return np.array([[sum_x/length, sum_y/length, sum_z/length]])


def extract_frontiers(mesh):
    frontiers = set()

    for face_id in range(0, mesh.num_faces):
        adj_faces = mesh.get_face_adjacent_faces(face_id)
        if len(adj_faces) <= 2:
            frontiers.add(face_id)

    return frontiers


def create_graph(mesh, centroids, normals, robot_pos,
                 traversal_tresh=35, bumpiness_tresh=0.37, dbscan_eps=3, dbscan_min_samples=2):
    """

    :param mesh:
    :param centroids:
    :param normals:
    :param closer_centroid_idx:
    :param traversal_tresh:
    :param dbscan_eps:
    :param dbscan_min_samples:
    :return:
    """
    print("Creating Graph... num faces:", mesh.num_faces)

    frontiers = extract_frontiers(mesh)
    print("Found ", frontiers, "frontiers")

    G = nx.Graph()

    for face_idx in xrange(mesh.num_faces):
        face = mesh.faces[face_idx]

        face_inclination = graph_search.MeshGraphSearch.calculate_traversal_angle(normals[face_idx])
        # if 0 <= face_inclination <= traversal_tresh or 180 - traversal_tresh <= face_inclination <= 180:
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

    #print "G edge_list:", len(list(G.edges())), sorted(list(G.edges()))

    # remove small connected components
    for component in list(nx.connected_components(G)):
        if len(component) < 3:
            for node in component:
                G.remove_node(node)

    g_centroids = [(centroids[v][0], centroids[v][1], centroids[v][2]) for v in sorted(G.nodes())]
    centroid_g_dict = {i: v for i, v in enumerate(sorted(G.nodes()))}

    closer_centroid_idx = mesh_planner.mesh_helper.find_closer_centroid(g_centroids, robot_pos,
                                                                        force_return_closer=True)
    conn_nodes = nx.node_connected_component(G, centroid_g_dict[closer_centroid_idx])
    Gconn = G.subgraph(conn_nodes).copy()

    kdtree = spatial.KDTree(g_centroids)
    pairs = kdtree.query_pairs(bumpiness_tresh)
    print "pairs:", len(pairs), pairs

    joined_by_bumpiness_nodes = set()
    for pair in pairs:
        p1_conn = centroid_g_dict[pair[0]]  # node inside the biggest connected component
        p2_out = centroid_g_dict[pair[1]]  # node outside of the biggest connected component

        # if edge is already mapped, then drop it
        if Gconn.has_edge(p1_conn, p2_out) or Gconn.has_edge(p2_out, p1_conn):
            continue

        # if edge is only connecting inside the Gconn drop it
        if Gconn.has_node(p1_conn) and Gconn.has_node(p2_out):
            continue

        # if edge is not connecting Gconn with other connected elements drop it
        if not Gconn.has_node(p1_conn) and not Gconn.has_node(p2_out):
            continue

        if p1_conn not in Gconn.nodes():
            p1_conn, p2_out = p2_out, p1_conn

        # if there is already a connection between the outside element and Gconn
        intersecting_gconn_nodes = list(set(G.neighbors(p2_out)).intersection(Gconn.nodes()))
        if len(intersecting_gconn_nodes) > 1:
            continue

        # this node is an another connected subgraph
        # add this node and the other ones of the subgraph
        # if not Gconn.has_node(p2_out):
        #     small_component = nx.node_connected_component(G, p2_out)
        #     for n in small_component:
        #         if not Gconn.has_node(n):
        #             Gconn.add_node(n)

        small_comp_nodes = nx.node_connected_component(G, p2_out)
        Gsmall = G.subgraph(small_comp_nodes).copy()
        GconnTemp = Gconn.copy()
        Gconn = nx.compose(Gconn, Gsmall)

        for pair2 in pairs:
            pair2_1 = centroid_g_dict[pair2[0]]
            pair2_2 = centroid_g_dict[pair2[1]]

            if (Gsmall.has_node(pair2_1) and GconnTemp.has_node(pair2_2)) or \
                (GconnTemp.has_node(pair2_1) and Gsmall.has_node(pair2_2)):

                gconn_node = pair2_1
                outside_node = pair2_2

                if not GconnTemp.has_node(gconn_node):
                    outside_node, gconn_node = gconn_node, outside_node

                # intersecting_gconn_nodes = list(set(Gconn.neighbors(gconn_node)).intersection(Gsmall.nodes()))
                # if len(intersecting_gconn_nodes) > 0:
                #     continue

                Gconn.add_edge(pair2_1, pair2_2, weight=1)
                joined_by_bumpiness_nodes.add(pair2_1)
                joined_by_bumpiness_nodes.add(pair2_2)

                # a = np.asarray(centroids[pair2_1])
                # b = np.asarray(centroids[pair2_2])
                #
                # print("dist:", np.linalg.norm(a - b))

    # add remaining edges of the new component from the original graph
    for e in G.edges():
        p1_conn = e[0]
        p2_out = e[1]

        # if edge is already mapped, then drop it
        if Gconn.has_edge(p1_conn, p2_out) or Gconn.has_edge(p2_out, p1_conn):
            continue

        # if edge is only connecting inside the Gconn drop it
        if not p1_conn in Gconn.nodes() and not p2_out in Gconn.nodes():
            continue

        Gconn.add_edge(p1_conn, p2_out, weight=1)

    print "Gconn node_list:", list(Gconn.nodes())

    # numpy array of x,y,z positions in sorted node order
    gcon_centroids = [(centroids[v][0], centroids[v][1], centroids[v][2]) for v in sorted(Gconn.nodes())]
    xyz = np.array(gcon_centroids)
    # scalar colors
    scalars = xyz[:, 2] #np.array(list(Gconn.nodes())) #xyz[:, 2]  #+ 5

    mlab.figure(1, bgcolor=(0, 0, 0))
    mlab.clf()

    pts = mlab.points3d(xyz[:, 0], xyz[:, 1], xyz[:, 2],
                        scalars,
                        scale_factor=0.1,
                        scale_mode='none',
                        colormap='Blues',
                        resolution=20)

    # estimate borders of the remainder graph
    border_centroids = []
    # for v in sorted(Gconn.nodes()):
    #     print "info:", v, nx.info(Gconn, v)
    #     if nx.degree(G, v) <= 8 and v not in joined_by_bumpiness_nodes:
    #         border_centroids.append((centroids[v][0], centroids[v][1], centroids[v][2]))

    # xyz_d2 = np.array(centroids_degree_2)
    # print "xyz_d2.shape:", xyz_d2.shape
    # scalars_d2 = np.ones(xyz_d2.shape[0])
    # pts2 = mlab.points3d(xyz_d2[:, 0], xyz_d2[:, 1], xyz_d2[:, 2],
    #                     scalars_d2,
    #                     scale_factor=0.2,
    #                     scale_mode='none',
    #                     color=(1.0, 0.0, 0.0),
    #                     resolution=20)

    centroid_gcon_dict = {v: int(i) for i, v in enumerate(gcon_centroids)}
    print "centroid_gcon_dict:", centroid_gcon_dict.keys()
    edge_list = []
    for e in Gconn.edges():
        e1 = (centroids[e[0]][0], centroids[e[0]][1], centroids[e[0]][2])
        e2 = (centroids[e[1]][0], centroids[e[1]][1], centroids[e[1]][2])
        edge_list.append([centroid_gcon_dict[e1], centroid_gcon_dict[e2]])

    edge_list = np.array(edge_list)
    #edge_list = np.array(list(Gconn.edges()))
    print "edge_list:", edge_list
    pts.mlab_source.dataset.lines = np.array(edge_list)
    #pts.update()
    lines = mlab.pipeline.stripper(pts)
    mlab.pipeline.surface(lines, color=(0.2, 0.4, 0.5), line_width=1.5, opacity=.9)  #colormap='Accent',

    # tube = mlab.pipeline.tube(pts, tube_radius=0.1)
    # mlab.pipeline.surface(tube, color=(0.8, 0.8, 0.8))

    #print "frontiers:", list(frontiers)
    intersecting_frontiers = list(set(frontiers).intersection(Gconn.nodes()))
    if len(intersecting_frontiers) > 0:
        centroids_frontiers = [centroids[v] for v in intersecting_frontiers]
        db = DBSCAN(eps=dbscan_eps, min_samples=dbscan_min_samples).fit(centroids_frontiers)
        core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
        core_samples_mask[db.core_sample_indices_] = True
        labels = db.labels_

        # Number of clusters in labels, ignoring noise if present.
        n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
        n_noise_ = list(labels).count(-1)

        rospy.loginfo('Estimated number of clusters: %d, noise_points: %s', n_clusters_, n_noise_)
        unique_labels = set(labels)
        colors = plt.cm.get_cmap('gist_rainbow', len(unique_labels))

        X = np.array(centroids_frontiers)
        for idx, label in enumerate(unique_labels):
            if label == -1:
                # Black used for noise.
                # col = [0, 0, 0, 1]
                continue
            else:
                col = colors(idx)

            #print "col", col

            class_member_mask = (labels == label)
            xyz = X[class_member_mask & core_samples_mask]
            #print "xyz:", xyz

            #fxyz = np.array([centroids[v] for v in intersecting_frontiers])
            pts2 = mlab.points3d(xyz[:, 0], xyz[:, 1], xyz[:, 2],
                                color=(col[0], col[1], col[2]),
                                scale_factor=0.15,
                                scale_mode='none',
                                resolution=20)

            # centroid = get_centroid_of_pts(xyz)
            # #print "centroid:", centroid
            # pts3 = mlab.points3d(centroid[:, 0], centroid[:, 1], centroid[:, 2],
            #                      color=(col[0], col[1], col[2]),
            #                      scale_factor=0.5,
            #                      scale_mode='none',
            #                      resolution=20)

    mlab.show()


if __name__ == '__main__':
    rospy.init_node('filter_mesh_node')
    rospy.loginfo("filter_mesh_node start")

    # test_files = [
    #     {"map": "map_01_frontiers.stl",
    #      "pos": (-4, 0, 0)},
    #     {"map": "map_02_stairs_cavelike.stl",
    #      "pos": (-4, -1.25, 0.5)},
    #     {"map": "map_03_narrow_passage.stl",
    #      "pos": (-4, -1.25, 0.5)},
    #     {"map": "map_03_narrow_passage_v2.stl",
    #      "pos": (-4, -1.25, 0.5)},
    #     {"map": "map_04_stairs_perfect.stl",
    #      "pos": (-4, -1.25, 0.5)},
    #     {"map": "map_05_cavelike.stl",
    #      "pos": (0, 0, 0)},
    # ]

    test_files = [
        {"map": "map_04_stairs_perfect.stl",
         "pos": (-4, -1.25, 0.5)}
    ]

    for test in test_files:
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


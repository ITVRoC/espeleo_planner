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
import open3d as o3d
from sklearn.neighbors import NearestNeighbors
import time
import scipy.sparse


rospack = rospkg.RosPack()
package_path = rospack.get_path('espeleo_planner')
scripts_path = os.path.join(package_path, "scripts")
sys.path.append(scripts_path)

import mesh_planner
from mesh_planner import graph_search


def create_graph(ply_filepath, robot_pos, traversal_tresh=35):
    print("Creating Graph... :", ply_filepath   )

    pcd = o3d.io.read_point_cloud(ply_path)
    downpcd = pcd.voxel_down_sample(voxel_size=0.30)

    start = time.time()
    filtered_points = []
    for i in range(len(downpcd.points)):
        normal = downpcd.normals[i]
        face_inclination = graph_search.MeshGraphSearch.calculate_traversal_angle(normal)
        if traversal_tresh < face_inclination < 180 - traversal_tresh:
            continue

        filtered_points.append(list(downpcd.points[i]))
    end = time.time()
    print("filter points:", end - start, "secs")

    start = time.time()
    k = 9  # number of neighbours you want
    neigh = NearestNeighbors(n_neighbors=k)
    neigh.fit(np.asarray(filtered_points))
    knn_matrix = neigh.kneighbors_graph(mode="distance").toarray()
    end = time.time()
    print("knn gen:", end - start, "secs")

    start = time.time()
    G = nx.Graph()

    for i in range(0, len(filtered_points)):
        G.add_node(i)

    (row, col, entries) = scipy.sparse.find(knn_matrix)
    for i in range(0, len(row)):
        a = np.array(filtered_points[row[i]])
        b = np.array(filtered_points[col[i]])
        dist = np.linalg.norm(a - b)

        if dist <= 0.61:
            G.add_edge(row[i], col[i], weight=entries[i])

    print "G node_list:", len(G.nodes())
    print "G edge_list:", len(G.edges())
    end = time.time()
    print("nx graph gen:", end - start, "secs")

    centroids = filtered_points

    g_centroids = [(centroids[v][0], centroids[v][1], centroids[v][2]) for v in sorted(G.nodes())]
    centroid_g_dict = {i: v for i, v in enumerate(sorted(G.nodes()))}

    closer_centroid_idx = mesh_planner.mesh_helper.find_closer_centroid(g_centroids, robot_pos,
                                                                        force_return_closer=True)
    conn_nodes = nx.node_connected_component(G, centroid_g_dict[closer_centroid_idx])
    Gconn = G.subgraph(conn_nodes).copy()
    Gconn_original = Gconn.copy()

    # estimate borders of the remainder graph
    border_centroids = []
    for v in sorted(Gconn.nodes()):
        if nx.degree(G, v) < 8:
            border_centroids.append((centroids[v][0], centroids[v][1], centroids[v][2]))

    print("border_centroids:", len(border_centroids))

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
    rospy.init_node('expand_borders_pcloud_node')
    rospy.loginfo("expand_borders_pcloud_node start")

    test_files = [
        {
            "ply": "riemannian_pcloud.ply",
            "dot": "riemannian_graph.dot",
            "robot_pos": (-15, 5, 0)
        }
    ]

    for test in test_files[:]:
        ply_path = os.path.join(package_path, "test", "point_cloud_planning", test["ply"])

        create_graph(ply_path, test["robot_pos"])

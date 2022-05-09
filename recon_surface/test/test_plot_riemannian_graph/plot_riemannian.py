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
import pcl
import pcl.pcl_visualization
import graphviz
import pydot
import pygraphviz as pgv
import inspect
import open3d as o3d
import math
from sklearn.neighbors import NearestNeighbors


rospack = rospkg.RosPack()
package_path = rospack.get_path('recon_surface')
scripts_path = os.path.join(package_path, "test")
sys.path.append(scripts_path)


def find_closer_centroid(centroids, p, tol=1.e-2, force_return_closer=False):
    """
    Find the centroid closer to P with a tolerance
    :param centroids: list of 3D points
    :param p: 3D point (x ,y, z)
    :param tol: tolerance
    :return:
    """

    source_face = -1
    min_dist = 999999999999

    #rospy.loginfo("p:%s centroid_size:%d", p, len(centroids))

    for idx, face_pt in enumerate(centroids):
        d = math.sqrt((face_pt[0] - p[0]) ** 2 + (face_pt[1] - p[1]) ** 2 + (face_pt[2] - p[2]) ** 2)

        # if force is True then return the closest face instead of the
        # best face point considering the tolerance parameter "tol"
        if d < min_dist:
            min_dist = d
            if force_return_closer:
                source_face = idx
                #rospy.loginfo("min_dist: %f", min_dist)

        if np.isclose(face_pt[0], p[0], rtol=tol, atol=tol) and \
            np.isclose(face_pt[1], p[1], rtol=tol, atol=tol) and \
                np.isclose(face_pt[2], p[2], rtol=tol, atol=tol):

            min_dist = d
            source_face = idx
            #rospy.loginfo("find direct tolerance: %f", min_dist)
            break

    #rospy.loginfo("Returned face idx:%f min_dist:%s", source_face, min_dist)
    return source_face


def plot_points(Gconn, centroids):
    #gcon_centroids = [(centroids[v][0], centroids[v][1], centroids[v][2]) for v in sorted(Gconn.nodes())]
    gcon_centroids = []
    for v in sorted(Gconn.nodes()):
        print v, centroids[v]
        c = (centroids[v][0], centroids[v][1], centroids[v][2])
        gcon_centroids.append(c)

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


def angle_between_vectors(v1, v2):
    """ Returns the angle in degrees between vectors 'v1' and 'v2'
    Original from: https://stackoverflow.com/questions/2827393/angles-between-two-n-dimensional-vectors-in-python

    >>> angle_between((1, 0, 0), (0, 1, 0))
    1.5707963267948966
    >>> angle_between((1, 0, 0), (1, 0, 0))
    0.0
    >>> angle_between((1, 0, 0), (-1, 0, 0))
    3.141592653589793

    :param v1: vector 1
    :param v2: vector 2
    :return:
    """
    def unit_vector(vector):
        """ Returns the unit vector of the vector.  """
        v_norm = np.linalg.norm(vector)
        if v_norm == 0.0:
            return np.asarray([0.0, 0.0])

        return vector / v_norm

    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)

    clipped_u = np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)
    arccos_u = np.arccos(clipped_u)
    return np.degrees(arccos_u)

def calculate_traversal_angle(face_normal, z_vector=(0, 0, -1)):
    """Public method Calculate the traversability of a node face
    Traversability is given by the inclination of the face with respect to the gravity vector (0, 0, -1)
    :param face_normal:
    :param z_vector:
    :return: traversal angle in degrees
    """
    theta = angle_between_vectors(face_normal, z_vector)

    # overcome inverted normals by normalizing the opposite angle (90-180) to 0-90
    if theta > 90:
        theta = math.fabs(theta - 180)

    return theta


def create_graph(ply_path, dot_path, robot_pos, traversal_tresh=35, bumpiness_tresh=0.37, dbscan_eps=3,
                 dbscan_min_samples=2):

    pcd = o3d.io.read_point_cloud(ply_path)
    print "pointcloud.has_normals:", bool(pcd.has_normals)
    #print "point:", np.asarray(pcd.points)
    #print "normal:", pcd.normals[0]
    #o3d.visualization.draw_geometries([pcd])

    downpcd = pcd.voxel_down_sample(voxel_size=0.25)
    print "downsampled normal:", downpcd.normals[0]
    #o3d.visualization.draw_geometries([downpcd])

    #cl, ind = downpcd.remove_radius_outlier(nb_points=16, radius=0.35)
    #cl, ind = downpcd.remove_statistical_outlier(nb_neighbors=20,std_ratio=2.0)
    #o3d.visualization.draw_geometries([cl])
    #display_inlier_outlier(downpcd, ind)

    filtered_points = []
    for i in range(len(downpcd.points)):
        normal = downpcd.normals[i]
        face_inclination = calculate_traversal_angle(normal)
        if traversal_tresh < face_inclination < 180 - traversal_tresh:
            continue

        filtered_points.append(list(downpcd.points[i]))

    filteredpcd = o3d.geometry.PointCloud()
    filteredpcd.points = o3d.utility.Vector3dVector(np.asarray(filtered_points))
    #o3d.visualization.draw_geometries([filteredpcd])

    # g_dot = graphviz.Source.from_file(dot_path)
    # print("Loading dotfile... ", g_dot)

    k = 6  # number of neighbours you want
    neigh = NearestNeighbors(n_neighbors=k)
    neigh.fit(np.asarray(filtered_points))
    knn_matrix = neigh.kneighbors_graph(mode="distance").toarray()

    #print "knn_matrix:", knn_matrix

    G = nx.Graph()
    for x in range(len(knn_matrix)):
        a = np.array(filtered_points[x])

        for y in range(len(knn_matrix[x])):
            if knn_matrix[x][y] != 0.0:
                b = np.array(filtered_points[y])
                dist = np.linalg.norm(a - b)

                if dist <= 0.45:
                    G.add_edge(x, y, weight=knn_matrix[x][y])


    # graphs = pydot.graph_from_dot_file(dot_path)
    # graph = graphs[0]
    # print("Loading dotfile... ", graph)

    # agraph = pgv.AGraph(dot_path)  # create a new graph from file
    # print("Loaded dotfile... num nodes:", agraph.number_of_nodes())
    # # print agraph.get_node("1")
    # #print agraph.edges("1")
    #
    # G = nx.Graph()
    # for node in agraph.nodes():
    #     u = int(node)
    #     G.add_node(u)
    #
    #     e_count = 0
    #     for edge in agraph.edges(node):
    #         v = int(edge[1])
    #         G.add_edge(u, v, weight=1)
    #
    #         e_count += 1
    #         if e_count >= 3:
    #             break
    #
    print "G node_list:", len(G.nodes())
    print "G edge_list:", len(G.edges())

    #g_centroids = [(filtered_points[v][0], filtered_points[v][1], filtered_points[v][2]) for v in sorted(G.nodes())]
    g_centroids = filtered_points
    centroid_g_dict = {i: v for i, v in enumerate(sorted(G.nodes()))}

    # remove small connected components
    for component in list(nx.connected_components(G)):
        if len(component) < 10:
            for node in component:
                G.remove_node(node)

    print "g_centroids:", len(g_centroids), g_centroids[0]

    closer_centroid_idx = find_closer_centroid(g_centroids, robot_pos, force_return_closer=True)
    conn_nodes = nx.node_connected_component(G, centroid_g_dict[closer_centroid_idx])
    Gconn = G.subgraph(conn_nodes).copy()

    kdtree = spatial.KDTree(g_centroids)
    pairs = kdtree.query_pairs(bumpiness_tresh)
    print "pairs:", len(pairs)

    f1 = mlab.figure("Borderless graph", bgcolor=(0, 0, 0))
    mlab.clf()
    plot_points(G, g_centroids)

    mlab.show()


if __name__ == '__main__':
    rospy.init_node('plot_riemannian_node')
    rospy.loginfo("plot_riemannian_node start")

    test_files = [
        {
            "ply": "riemannian_pcloud.ply",
            "dot": "riemannian_graph.dot",
            "robot_pos": (-15, 5, 0)
        }
    ]

    for test in test_files[:]:
        ply_path = os.path.join(package_path, "test", "test_plot_riemannian_graph", test["ply"])
        dot_path = os.path.join(package_path, "test", "test_plot_riemannian_graph", test["dot"])

        create_graph(ply_path, dot_path, test["robot_pos"])


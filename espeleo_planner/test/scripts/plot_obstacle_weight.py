#!/usr/bin/env python

import sys
import os
import math
import rospkg
import rospy
import pymesh
import networkx as nx
import numpy as np
from mayavi import mlab
from scipy import spatial
import matplotlib.pyplot as plt

rospack = rospkg.RosPack()
package_path = rospack.get_path('espeleo_planner')
scripts_path = os.path.join(package_path, "scripts")
sys.path.append(scripts_path)

# remove faces
# https://github.com/PyMesh/PyMesh/issues/118

def weight_border(u, obstacle_kdtree, d0=6.0, c1=2.0, min_dist=0.1):
    """Calculate the weight based on distance from border nodes
    This weight aims to penalize paths closer to dangerous areas such as map borders and obstacles
    Using repulsive potential fields https://youtu.be/MQjeqvbzhGQ?t=222
    d0 = mininum_distance to evaluate
    c1 = scale constant
    obstacle_d = distance to the closest obstacle

    if obstacle_d <= d0:
        c1 * (1/obstacle_d - 1/d0)^2
    else:
        0

    :param target: id of the target node
    :return:
    """
    distances, nearest_idx = obstacle_kdtree.query([u])
    obstacle_d = distances[0]

    if obstacle_d <= min_dist:
        # treating zero as a very close number
        obstacle_d = min_dist

    if obstacle_d <= d0:
        repulsive_w = 1/2.0 * c1 * (((1 / float(obstacle_d)) - (1 / float(d0))) ** 2)
    else:
        repulsive_w = 0

    print "distances:", distances, "obstacle_d:", obstacle_d, "repulsive_w:", repulsive_w

    return repulsive_w

def weight_border2(u, centroids_degree_2, d0=2.0, c1=3, min_dist=0.2):
    """Calculate the weight based on distance from border nodes
    This weight aims to penalize paths closer to dangerous areas such as map borders and obstacles
    Using repulsive potential fields https://youtu.be/MQjeqvbzhGQ?t=222
    d0 = mininum_distance to evaluate
    c1 = scale constant
    obstacle_d = distance to the closest obstacle

    if obstacle_d <= d0:
        c1 * (1/obstacle_d - 1/d0)^2
    else:
        0

    :param target: id of the target node
    :return:
    """
    obstacle_d = float('inf')

    for k, v in enumerate(centroids_degree_2):
        a = np.asarray(u)
        b = np.asarray(v)

        dist = np.linalg.norm(a-b)
        if dist < obstacle_d:
            obstacle_d = dist

    if obstacle_d <= min_dist:
        # treating zero as a very close number
        obstacle_d = min_dist

    if obstacle_d <= d0:
        repulsive = c1 * (((1 / float(obstacle_d)) - (1 / float(d0))) ** 2)
    else:
        repulsive = 0

    #print "distances:", distances, "nearest_idx:", nearest_idx, "repulsive:", repulsive

    return repulsive

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

def find_closer_centroid(centroids, p):
    """
    Find the centroid closer to P
    :param centroids: list of 3D points
    :param p: 3D point (x ,y, z)
    :param tol: tolerance
    :return:
    """

    source_face = -1
    min_dist = 999999999999

    rospy.loginfo("p:%s centroid_size:%d", p, len(centroids))

    for idx, face in enumerate(centroids):
        d = math.sqrt((face[0] - p[0]) ** 2 + (face[1] - p[1]) ** 2 + (face[2] - p[2]) ** 2)

        if d < min_dist:
            min_dist = d
            source_face = idx

    rospy.loginfo("Returned face idx: %f min_dist: %f %s", source_face, min_dist, centroids[source_face])
    return source_face

def angle_between_vectors2(v1, v2):
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

def terrain_weight(source_normal_p):
    z_axe = [0, 0, -1]  # vector representing the Z axe
    scalar = np.dot(source_normal_p, z_axe)  # scalar product between the face normal and Z axe
    norms = np.linalg.norm(source_normal_p) * np.linalg.norm(z_axe)  # norm between face normal and Z Axe
    cos_angle = scalar / norms

    radians = math.acos(cos_angle)  # arccos of scalar/norms
    angle_degrees = math.degrees(radians)
    #angle_degrees = math.fabs(angle_degrees)

    return angle_degrees


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
    print("Found ", mesh.num_faces, "frontiers")

    G = nx.Graph()

    for face_idx in xrange(mesh.num_faces):
        face = mesh.faces[face_idx]

        face_inclination = terrain_weight(normals[face_idx])
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

    # print "G node_list:", len(list(G.nodes())), sorted(list(G.nodes()))
    # print "G edge_list:", len(list(G.edges())), sorted(list(G.edges()))

    # remove small connected components
    for component in list(nx.connected_components(G)):
        if len(component) < 3:
            for node in component:
                G.remove_node(node)

    g_centroids = [(centroids[v][0], centroids[v][1], centroids[v][2]) for v in sorted(G.nodes())]
    centroid_g_dict = {i: v for i, v in enumerate(sorted(G.nodes()))}

    closer_centroid_idx = find_closer_centroid(g_centroids, robot_pos)
    conn_nodes = nx.node_connected_component(G, centroid_g_dict[closer_centroid_idx])
    Gconn = G.subgraph(conn_nodes).copy()

    gcon_centroids = [(centroids[v][0], centroids[v][1], centroids[v][2]) for v in sorted(Gconn.nodes())]
    xyz = np.array(gcon_centroids)
    scalars = xyz[:, 2] #np.array(list(Gconn.nodes())) #xyz[:, 2]  #+ 5

    # estimate borders of the remainder graph
    # border_nodes = [Gconn.info(v) for v in sorted(Gconn.nodes())]
    centroids_degree_2 = []
    for v in sorted(Gconn.nodes()):
        #print "info:", v, nx.info(Gconn, v)
        if nx.degree(G, v) <= 9:
            centroids_degree_2.append((centroids[v][0], centroids[v][1], centroids[v][2]))

    border_kdtree = spatial.KDTree(centroids_degree_2)

    mlab.figure(1, bgcolor=(0, 0, 0))
    mlab.clf()

    # pts = mlab.points3d(xyz[:, 0], xyz[:, 1], xyz[:, 2],
    #                     scalars,
    #                     scale_factor=0.1,
    #                     scale_mode='none',
    #                     colormap='Blues',
    #                     resolution=20)
    #
    # # estimate borders of the remainder graph
    # # border_nodes = [Gconn.info(v) for v in sorted(Gconn.nodes())]
    # centroids_degree_2 = []
    # for v in sorted(Gconn.nodes()):
    #     #print "info:", v, nx.info(Gconn, v)
    #     if nx.degree(G, v) <= 2:
    #         centroids_degree_2.append((centroids[v][0], centroids[v][1], centroids[v][2]))
    #
    xyz_d2 = np.array(centroids_degree_2)
    #print "xyz_d2.shape:", xyz_d2.shape
    scalars_d2 = np.ones(xyz_d2.shape[0])
    pts2 = mlab.points3d(xyz_d2[:, 0], xyz_d2[:, 1], xyz_d2[:, 2],
                        scalars_d2,
                        scale_factor=0.2,
                        scale_mode='none',
                        color=(1.0, 0.0, 0.0),
                        resolution=20)
    #
    #
    # centroid_gcon_dict = {v: int(i) for i, v in enumerate(gcon_centroids)}
    # print "centroid_gcon_dict:", centroid_gcon_dict.keys()
    # edge_list = []
    # for e in Gconn.edges():
    #     e1 = (centroids[e[0]][0], centroids[e[0]][1], centroids[e[0]][2])
    #     e2 = (centroids[e[1]][0], centroids[e[1]][1], centroids[e[1]][2])
    #     edge_list.append([centroid_gcon_dict[e1], centroid_gcon_dict[e2]])
    #
    # edge_list = np.array(edge_list)
    # #edge_list = np.array(list(Gconn.edges()))
    # print "edge_list:", edge_list
    # pts.mlab_source.dataset.lines = np.array(edge_list)
    # #pts.update()
    # lines = mlab.pipeline.stripper(pts)
    # mlab.pipeline.surface(lines, color=(0.2, 0.4, 0.5), line_width=1, opacity=.4)  #colormap='Accent',

    # tube = mlab.pipeline.tube(pts, tube_radius=0.1)
    # mlab.pipeline.surface(tube, color=(0.8, 0.8, 0.8))

    triangles = [mesh.faces[n] for n in sorted(Gconn.nodes())]
    xyz = mesh.vertices

    wire_mesh = mlab.triangular_mesh(xyz[:, 0], xyz[:, 1], xyz[:, 2], triangles,
                                representation='wireframe',
                                opacity=0)

    f = [weight_border(centroids[n], border_kdtree) for n in sorted(Gconn.nodes())]
    print sorted(f)
    #f = [weight_border2(centroids[n], centroids_degree_2, d0=3.0, c1=1, min_dist=0.2) for n in sorted(Gconn.nodes())]
    wire_mesh.mlab_source.dataset.cell_data.scalars = f
    wire_mesh.mlab_source.dataset.cell_data.scalars.name = "Cell data"
    wire_mesh.mlab_source.update()
    mesh2 = mlab.pipeline.set_active_attribute(wire_mesh, cell_scalars="Cell data")

    # #f = [weight_border(v, border_kdtree, d0=3.0, c1=1, min_dist=0.2) for v in mesh.vertices]
    # f = [weight_border2(v, centroids_degree_2, d0=3.0, c1=1, min_dist=0.8) for v in mesh.vertices]
    # wire_mesh.mlab_source.dataset.point_data.scalars = f
    # wire_mesh.mlab_source.dataset.point_data.scalars.name = "Point data"
    # wire_mesh.mlab_source.update()
    # mesh2 = mlab.pipeline.set_active_attribute(wire_mesh, point_scalars='Point data')

    mlab.pipeline.surface(mesh2, colormap='jet')

    # #mlab.savefig('/tmp/mayavi2_spring.png')
    mlab.show() # interactive window

    # plot 2D representation of the function for this map
    fig = plt.figure()
    ax = plt.axes(projection='3d')

    xyz_gconn = np.asarray([centroids[n] for n in sorted(Gconn.nodes())])
    ax.plot_trisurf(xyz_gconn[:, 0], xyz_gconn[:, 1], f, cmap='viridis', edgecolor='none')
    ax.set_title('Surface plot')
    plt.show()


if __name__ == '__main__':
    rospy.init_node('obstacle_weight_node')
    rospy.loginfo("obstacle_weight start")

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


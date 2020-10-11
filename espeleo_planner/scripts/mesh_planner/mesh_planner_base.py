#!/usr/bin/env python

import rospy
import pymesh
import networkx as nx
import multiprocessing
import graph_search
import numpy as np
import time
from scipy import spatial
import mesh_helper
from sklearn.cluster import DBSCAN


class MeshPlannerBase:
    """
    Mesh Path Finder given a mesh, a list of metrics and a source and destination points
    calculates the optimum paths
    """

    def __init__(self, mesh_path, graph_metrics_types):
        """
        Mesh Path Finder constructor
        :param mesh_path: path to mesh .stl file
        :param graph_metrics_types: list of graph metrics types to calculate (GraphMetricType enum object)
        """

        self.mesh_path = mesh_path
        self.mesh = pymesh.load_mesh(self.mesh_path)

        if isinstance(graph_metrics_types, (list, tuple)):
            self.graph_metrics_types = graph_metrics_types
        elif graph_metrics_types is not None:
            self.graph_metrics_types = [self.graph_metrics_types]
        else:
            raise TypeError("graph_metrics is not a valid object type [list, tuple]")

        self.transversality_threshold = 30  # max inclination (in degrees) the robot could climb
        self.bumpiness_threshold = 0.5  # maximum bump the robot could jump between surfaces TODO add reference here
        self.border_threshold = 0.3  # distance to expand from borders to other face centroids

        self.shortest_comb_weight = 0.25  # this is a shortest weight to combine the weights of the metrics
        self.energy_comb_weight = 0.25  # this is a energy weight to combine the weights of the metrics
        self.transversality_comb_weight = 0.50  # this is a transversality weight to combine the weights of the metrics

        self.mesh.enable_connectivity()  # enables connectivity on mesh
        self.mesh.add_attribute("face_centroid")  # adds the face centroids to be accessed
        self.mesh.add_attribute("face_normal")  # adds the face normals to be accessed

        self.faces = self.mesh.faces
        self.centroids = self.mesh.get_face_attribute("face_centroid")
        self.normals = self.mesh.get_face_attribute("face_normal")

        self.mesh_frontiers = set()

        rospy.loginfo("Vertex and Face count: %d, %d" % (self.mesh.num_vertices, self.mesh.num_faces))
        rospy.loginfo("Dimensions and Vertexes in a face: %d, %d" % (self.mesh.dim, self.mesh.vertex_per_face))

    def plot_graph_3d(self, G, title=None, source_id=None, target_id=None, border_3d_points=None,
                      reachable_frontiers_ids=None, frontier_centroids_ids=None, frontier_visit_ids=None):
        """Plot the 3D graph using Mayavi (useful for debugging)

        :param G: the NetorkX graph
        :param title: window title
        :param source_id: source node id
        :param target_id: target node id
        :param border_3d_points: mesh borders points
        :param reachable_frontiers_ids: frontier node ids
        :param frontier_centroids_ids: frontier centroids ids
        :param frontier_visit_ids: the visit point for the frontiers (generally is the closest point to the robot)
        :return:
        """
        from mayavi import mlab

        if not title:
            title = 1

        mlab.figure(title, bgcolor=(0, 0, 0))
        mlab.clf()

        g_centroids = [tuple(self.centroids[v]) for v in sorted(G.nodes())]
        centroid_gcon_dict = {v: int(i) for i, v in enumerate(g_centroids)}

        xyz = np.array(g_centroids)
        scalars = xyz[:, 2]
        pts = mlab.points3d(xyz[:, 0], xyz[:, 1], xyz[:, 2],
                            scalars,
                            scale_factor=0.1,
                            scale_mode='none',
                            colormap='Blues',
                            resolution=20)

        edge_list = []
        for e in G.edges():
            e1 = tuple(self.centroids[e[0]])
            e2 = tuple(self.centroids[e[1]])
            edge_list.append([centroid_gcon_dict[e1], centroid_gcon_dict[e2]])

        edge_list = np.array(edge_list)
        pts.mlab_source.dataset.lines = np.array(edge_list)
        # lines = mlab.pipeline.stripper(pts)
        mlab.pipeline.surface(pts, color=(0.2, 0.4, 0.5), line_width=1, opacity=.4)

        if border_3d_points and len(border_3d_points) > 0:
            xyz_d2 = np.array(border_3d_points)
            scalars_d2 = np.ones(xyz_d2.shape[0])
            mlab.points3d(xyz_d2[:, 0], xyz_d2[:, 1], xyz_d2[:, 2], scalars_d2,
                          scale_factor=0.1,
                          scale_mode='none',
                          color=(1.0, 0.0, 0.0),
                          resolution=20)

        # add source and target labels
        if source_id:
            src_3d = self.centroids[source_id]
            mlab.text(src_3d[0], src_3d[1], "source", z=src_3d[2], width=0.2)
            mlab.points3d([src_3d[0]], [src_3d[1]], [src_3d[2]],
                          scale_factor=0.25,
                          scale_mode='none',
                          color=(0, 1.0, 0.1),
                          resolution=20)

        if target_id:
            tgt_3d = self.centroids[target_id]
            mlab.text(tgt_3d[0], tgt_3d[1], "target", z=tgt_3d[2], width=0.2)
            mlab.points3d([tgt_3d[0]], [tgt_3d[1]], [tgt_3d[2]],
                          scale_factor=0.25,
                          scale_mode='none',
                          color=(0, 0.1, 1.0),
                          resolution=20)

        if reachable_frontiers_ids and len(reachable_frontiers_ids) > 0:
            frontiers_3dp = [tuple(self.centroids[v]) for v in reachable_frontiers_ids]
            xyz = np.array(frontiers_3dp)
            mlab.points3d(xyz[:, 0], xyz[:, 1], xyz[:, 2],
                          scale_factor=0.15,
                          scale_mode='none',
                          color=(1.0, 0.1, 1.0),
                          resolution=20)

        if frontier_centroids_ids and len(frontier_centroids_ids) > 0:
            centroids_3dp = [tuple(self.centroids[v]) for v in frontier_centroids_ids]
            xyz = np.array(centroids_3dp)
            mlab.points3d(xyz[:, 0], xyz[:, 1], xyz[:, 2],
                          scale_factor=0.35,
                          scale_mode='none',
                          color=(1.0, 0.1, 1.0),
                          resolution=20)

        if frontier_visit_ids and len(frontier_visit_ids) > 0:
            centroids_3dp = [tuple(self.centroids[v]) for v in frontier_visit_ids]
            xyz = np.array(centroids_3dp)
            mlab.points3d(xyz[:, 0], xyz[:, 1], xyz[:, 2],
                          scale_factor=0.35,
                          scale_mode='none',
                          color=(1.0, 0.1, 1.0),
                          resolution=20)

        mlab.show()

    def extract_frontiers_from_mesh(self):
        """Extract the face frontiers directly from the mesh file

        :return: a set with the ids of the frontier faces
        """
        for face_id in range(0, self.mesh.num_faces):
            adj_faces = self.mesh.get_face_adjacent_faces(face_id)
            if len(adj_faces) <= 2:
                self.mesh_frontiers.add(face_id)

        return self.mesh_frontiers

    def create_graph_from_mesh(self):
        """Create a graph from the mesh's faces centroids conecting nodes using the conectivity graph of the
        original mesh
        :return: a networkx graph G
        """
        G = nx.Graph()
        for face_idx in xrange(self.mesh.num_faces):
            G.add_node(face_idx)

        # add edges for adjacent faces
        for face_idx in list(G.nodes()):
            face_vertexes = self.mesh.faces[face_idx]
            for v in face_vertexes:
                vertex_adj_faces = self.mesh.get_vertex_adjacent_faces(v)
                for face_adjacent in vertex_adj_faces:
                    if face_adjacent != face_idx and G.has_node(face_adjacent):
                        G.add_edge(face_idx, face_adjacent, weight=1)

        return G

    def prepare_graph(self, G, source_id, target_id=None):
        """Filter and extract frontiers given a mesh graph. Remove outliers, join nearby traversable surfaces,
        perform a border expansion to prevent collisions, etc.

        :param G:
        :param source_id: source node id
        :param target_id: target node id
        :return: G, f_centroids_ids, filtered_reachable_frontiers
        """
        G = self.filter_graph_by_traversable_faces(G)
        G = self.remove_non_connected_components(G, source_id)

        mesh_frontiers = self.extract_frontiers_from_mesh()
        graph_frontiers = self.extract_borders_from_graph(G, degree_tresh=12)
        reachable_frontiers = mesh_frontiers.intersection(graph_frontiers)

        G = self.expand_graph_borders(G)

        # add important nodes that could be lost in previous filtering steps
        checked_nodes = list(reachable_frontiers)
        unchecked_nodes = [source_id]
        if target_id:
            unchecked_nodes.append(target_id)

        G = self.reconnect_non_removable_nodes(G,
                                               checked_nodes,
                                               unchecked_nodes=unchecked_nodes,
                                               max_distance=self.border_threshold + 1.0)
        G = self.remove_non_connected_components(G, source_id)

        filtered_reachable_f_ids = reachable_frontiers.intersection(G.nodes())
        f_centroids_ids = []
        f_visit_ids = []
        if len(filtered_reachable_f_ids) > 0:
            f_visit_ids, f_centroids_ids, f_centroids, f_points = self.cluster_frontier_borders(G,
                                                                                                filtered_reachable_f_ids,
                                                                                                source_id)

        return G, f_centroids_ids, f_centroids_ids, filtered_reachable_f_ids

    def extract_borders_from_graph(self, G, degree_tresh=9):
        """Extract the nodes that has a degree less than two, this is a heuristic to detect which nodes
        are located at the edges of the graph (such as obstacle borders and map border limits)

        :param G:
        :param degree_tresh: all nodes with degree less than degree_tresh are considered as border
        :return: list of border node indices
        """
        border_nodes = []
        for v in sorted(G.nodes()):
            if nx.degree(G, v) <= degree_tresh:
                border_nodes.append(v)

        return border_nodes

    def filter_graph_by_traversable_faces(self, G):
        """Remove non traversable faces from the graph
        CAUTION: this can return a non fully connected graph with multiple
        smaller subgraphs

        :param G:
        :return: a graph with only traversable faces
        """
        for face_idx in list(G.nodes()):
            face_inclination = graph_search.MeshGraphSearch.calculate_traversal_angle(self.normals[face_idx])
            if face_inclination > self.transversality_threshold:
                G.remove_node(face_idx)

        return G

    def expand_graph_borders(self, G):
        """Remove nodes from a graph that are withing a distance treshold from the borders
        this helps to generate routes where the robot could actually move (narrow passages) and prevent
        collisions with obstacles and "static falls"

        :param G:
        :return: a smaller graph G' with the expanded borders removed
        """
        # estimate borders of the remainder graph
        border_centroids = []
        for v in sorted(G.nodes()):
            if nx.degree(G, v) <= 9:
                border_centroids.append(tuple(self.centroids[v]))  # tuples are hashable! lists are not

        # remove nodes from graph that are near to the borders
        # given a distance threshold
        border_kdtree = spatial.KDTree(border_centroids)
        for v in list(G.nodes()):
            point = self.centroids[v]
            distances, nearest_idx = border_kdtree.query([point])
            obstacle_d = distances[0]
            if obstacle_d <= self.border_threshold:
                G.remove_node(v)

        # remove small connected components
        for component in list(nx.connected_components(G)):
            if len(component) < 3:
                for node in component:
                    G.remove_node(node)

        return G

    @staticmethod
    def remove_non_connected_components(G, source):
        """Remove all unconnected components not connected to the source node (position of the robot)
    
        :param G: 
        :param source: 
        :return: smaller G with the non connected components removed
        """
        conn_nodes = nx.node_connected_component(G, source)
        return G.subgraph(conn_nodes).copy()

    def reconnect_non_removable_nodes(self, G, checked_nodes, unchecked_nodes=None, max_distance=1.0):
        """Add non removable nodes to the graph which can be deleted by
        previous filtering algorithms such as the source and destination points
        The checked_nodes list will be added to the graph after validation for a maximum distance established by the
        max_distance parameter. The unchecked_nodes will be added without previous validation, usefull for the start
        and end nodes.

        :param G: graph object
        :param checked_nodes: nodes to check against a maximum distance threshold
        :param unchecked_nodes:
        :param max_distance:
        :return: G with important nodes and edges added to it
        """
        if not unchecked_nodes:
            unchecked_nodes = []

        # check if the source, the target, and the frontiers are reachable
        if not all(G.has_node(n_idx) for n_idx in checked_nodes):
            borderless_g_centroids = [tuple(self.centroids[v]) for v in sorted(G.nodes())]
            assert len(borderless_g_centroids) > 0, "The expand borders function did not leave any nodes, maybe" \
                                                    "the treshold is too high? " \
                                                    "border_tresh:{}".format(self.border_threshold)

            borderless_g_dict = {i: v for i, v in enumerate(sorted(G.nodes()))}
            borderless_kdtree = spatial.KDTree(borderless_g_centroids)

            for n_idx in checked_nodes + unchecked_nodes:
                if G.has_node(n_idx):
                    continue

                d, nearest_idx = borderless_kdtree.query(self.centroids[n_idx])
                nearest_idx = int(nearest_idx)
                # print "n_idx:", n_idx, "distances:", d, "nearest_idx:", nearest_idx, \
                #     "checked_nodes:", checked_nodes, "uncheched_nodes:", uncheched_nodes

                if d <= max_distance or n_idx in unchecked_nodes:
                    G.add_node(n_idx)
                    G.add_edge(n_idx, borderless_g_dict[nearest_idx])

        return G

    def cluster_frontier_borders(self, G, reachable_frontiers, source_id, dbscan_eps=2.5, dbscan_min_samples=1):
        """From a list of frontier borders, label them in clusters based on distance and extract the
        cluster centroids

        :param G: graph object
        :param reachable_frontiers: list of frontier node ids
        :param source_id: the node id of the robot position
        :param dbscan_eps: maximum distance between two points
        :param dbscan_min_samples: min number of points to became a cluster
        :return: list of visit nodes ids,  list of centroid node ids, list of centroids points, list of frontier points by cluster
        """

        def get_centroid_of_pts(arr):
            """ Get centroid of a list of 3D points
            :param arr: numpy arr of 3D points
            :return: centroid 3D point
            """
            length = arr.shape[0]
            sum_x = np.sum(arr[:, 0])
            sum_y = np.sum(arr[:, 1])
            sum_z = np.sum(arr[:, 2])
            return np.array([[sum_x / length, sum_y / length, sum_z / length]])

        reachable_frontiers_points = [self.centroids[v] for v in reachable_frontiers]
        db = DBSCAN(eps=dbscan_eps, min_samples=dbscan_min_samples).fit(reachable_frontiers_points)
        core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
        core_samples_mask[db.core_sample_indices_] = True
        labels = db.labels_

        # Number of clusters in labels, ignoring noise if present.
        n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
        n_noise_ = list(labels).count(-1)

        rospy.loginfo('Estimated number of clusters: %d, noise_points: %s', n_clusters_, n_noise_)
        unique_labels = set(labels)

        reachable_node_3d_points = [tuple(self.centroids[n_id]) for n_id in sorted(G.nodes())]
        pos_tuple_to_node_id_dict = {tuple(self.centroids[n_id]): n_id for n_id in G.nodes()}

        frontier_cluster_closest_id = []
        frontier_cluster_visit_points_id = []
        frontier_cluster_centers = []
        frontier_cluster_points = []

        X = np.array(reachable_frontiers_points)
        for idx, label in enumerate(unique_labels):
            if label == -1:
                # -1 == noise.
                continue

            class_member_mask = (labels == label)
            xyz = X[class_member_mask & core_samples_mask]

            closest_id = mesh_helper.find_closer_centroid(xyz,
                                                          tuple(self.centroids[source_id]),
                                                          force_return_closer=True)

            # closest frontier node to the robot
            tuple_xyz = list(map(tuple, xyz))
            visit_point_id = pos_tuple_to_node_id_dict[tuple_xyz[closest_id]]

            # centroid of the cluster
            centroid = get_centroid_of_pts(xyz)
            centroid_list_id = mesh_helper.find_closer_centroid(reachable_node_3d_points,
                                                                tuple(centroid[0]),
                                                                force_return_closer=True)
            centroid_node_id = pos_tuple_to_node_id_dict[reachable_node_3d_points[centroid_list_id]]

            frontier_cluster_closest_id.append(centroid_node_id)
            frontier_cluster_centers.append(self.centroids[centroid_node_id])
            frontier_cluster_visit_points_id.append(visit_point_id)
            frontier_cluster_points.append(xyz)

        return frontier_cluster_visit_points_id, frontier_cluster_closest_id, frontier_cluster_centers, \
               frontier_cluster_points

    def run_graph_process(self, graph_metric_type, source_id, target_id, return_dict, is_debug=False):
        """Generate a graph and run the path planning using a metric

        :param graph_metric_type: the metric type to use in this graph process
        :param source_id: source node id
        :param target_id: target node id
        :param return_dict: the shared variable to send and receive data from processes
        :param is_debug: a flag to define some debug parameters such as 3D plot of the connectivity graph
        :return: Nothing, it uses the return_dict variable for this
        """
        rospy.loginfo("Started graph process: %s", graph_metric_type.name)
        start_time = time.clock()

        # graph creation and filtering
        G = self.create_graph_from_mesh()
        G, f_visit_ids, f_centroids_ids, filtered_reachable_frontiers = self.prepare_graph(G, source_id,
                                                                                           target_id=target_id)

        if is_debug:
            self.plot_graph_3d(G,
                               title=graph_metric_type.name,
                               source_id=source_id,
                               target_id=target_id,
                               reachable_frontiers_ids=list(filtered_reachable_frontiers),
                               frontier_centroids_ids=f_centroids_ids,
                               frontier_visit_ids=f_visit_ids)

        g_search = graph_search.MeshGraphSearch(G,
                                                graph_metric_type,
                                                self.centroids,
                                                self.normals,
                                                c_short=self.shortest_comb_weight,
                                                c_energy=self.energy_comb_weight,
                                                c_traversal=self.transversality_comb_weight)

        (length, path) = g_search.dijkstra_search({source_id}, target_id)
        rospy.loginfo("Ended process: %s %.2f seconds", graph_metric_type.name, time.clock() - start_time)

        if path is not None:
            return_dict[graph_metric_type] = g_search
        else:
            # could not get any path for the pair source/target
            return_dict[graph_metric_type] = None

    def run(self, source_id, target_id, is_multithread=False):
        """Run the graphs using the provided metrics and returns the path list
        :param source_id: node source id
        :param target_id: node target id
        :param is_multithread: a flag to define if the processes are going to be executed in threads (processes) or not
        :return:
        """
        processes_list = []
        manager = multiprocessing.Manager()
        return_dict = manager.dict()

        if is_multithread:
            # run the processes simultaneously
            for gmt in self.graph_metrics_types:
                graph_proc = multiprocessing.Process(target=self.run_graph_process,
                                                     args=[gmt, source_id, target_id, return_dict])
                graph_proc.start()
                processes_list.append(graph_proc)

            for process in processes_list:
                process.join()
        else:
            # run processes sequentially
            for gmt in self.graph_metrics_types:
                self.run_graph_process(gmt, source_id, target_id, return_dict)

        # prepare result from face id to world point
        world_path_dict = {}
        for gmt in self.graph_metrics_types:
            p_finder = return_dict[gmt]
            metric_name = gmt.name

            if p_finder is None:
                rospy.logerr("Cannot find path for metric: %s", metric_name)
                continue

            p_finder.print_path_metrics()

            p_list = [self.centroids[f_id] for f_id in p_finder.get_path()]
            world_path_dict[gmt] = {'path': p_list, 'cost': p_finder.get_path_distance()}

        return world_path_dict

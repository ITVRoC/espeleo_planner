#!/usr/bin/env python

import rospy
import networkx as nx
import multiprocessing
from . import graph_search
from . import rrt_graph_search
import numpy as np
from scipy import spatial
import traceback
import time

from sklearn.neighbors import NearestNeighbors
import scipy.sparse


class PointCloudPlannerBase:
    """
    Path Finder given a point cloud with normals, a list of metrics and a source and destination points
    calculates the optimum paths
    """

    def __init__(self, pcloud, graph_metrics_types, voxel_size=0.30):
        """
        Point Cloud Path Finder constructor
        :param mesh_path: path to point cloud .ply file
        :param graph_metrics_types: list of graph metrics types to calculate (GraphMetricType enum object)
        """

        self.pcd = pcloud

        if isinstance(graph_metrics_types, (list, tuple)):
            self.graph_metrics_types = graph_metrics_types
        elif graph_metrics_types is not None:
            self.graph_metrics_types = [self.graph_metrics_types]
        else:
            raise TypeError("graph_metrics is not a valid object type [list, tuple]")

        # REAL ROBOT CONSTANTS
        # self.transversality_threshold = 40  # REAL ROBOT
        # self.border_threshold = 0.4 # REAL ROBOT

        # SIMULATED ROBOT CONSTANTS
        self.transversality_threshold = 25  # max inclination (in degrees) the robot could climb
        self.bumpiness_threshold = 0.5  # maximum bump the robot could jump between surfaces TODO add reference here
        self.border_threshold = 0.45  # distance to expand from borders to other face centroids

        # self.shortest_comb_weight = 0.80
        # self.energy_comb_weight = 0.10
        # self.transversality_comb_weight = 0.10
        self.shortest_comb_weight = 0.25  # this is a shortest weight to combine the weights of the metrics
        self.energy_comb_weight = 0.25  # this is a energy weight to combine the weights of the metrics
        self.transversality_comb_weight = 0.50  # this is a transversality weight to combine the weights of the metrics

        self.centroids = self.pcd.points
        self.normals = self.pcd.normals

        self.mesh_frontiers = set()

        rospy.loginfo("[Planner] Point cloud size: %d" % len(self.centroids))

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

    def create_graph_from_pcloud(self, k=9, max_euclidean_dist=0.61):
        """Create a graph from the the points using a k-nn euclidean graph
        :return: a networkx graph G
        """
        start_time = time.process_time()
        neigh = NearestNeighbors(n_neighbors=k, radius=0.30) 
        neigh.fit(np.array(self.centroids))
        knn_matrix = neigh.kneighbors_graph(mode="distance")
        #print("time NearestNeighbors:", (time.process_time() - start_time))

        # G = nx.Graph()
        #
        # for i in range(0, len(self.centroids)):
        #     G.add_node(i)

        start_time = time.process_time()
        # (row, col, entries) = scipy.sparse.find(knn_matrix)

        G = nx.from_scipy_sparse_matrix(knn_matrix, parallel_edges=False, create_using=None, edge_attribute='weight')

        # for i in range(0, len(row)):
        #     G.add_edge(row[i], col[i], weight=entries[i])

        # for i in range(0, len(row)):
        #     a = np.array(self.centroids[row[i]])
        #     b = np.array(self.centroids[col[i]])
        #     dist = np.linalg.norm(a - b)
        #
        #     if dist <= max_euclidean_dist:
        #         G.add_edge(row[i], col[i], weight=entries[i])

        #print("time add edges:", (time.process_time() - start_time))

        return G

    def prepare_graph(self, G, source_id, target_id=None):
        """Filter and extract frontiers given a pcloud graph. Remove outliers, join nearby traversable surfaces,
        perform a border expansion to prevent collisions, etc.

        :param G:
        :param source_id: source node id
        :param target_id: target node id
        :return: G, f_centroids_ids, filtered_reachable_frontiers
        """
        # print "G size:", len(G.nodes)
        #G = self.filter_graph_by_traversable_faces(G)

        #print("G size:", len(G.nodes))
        G = self.remove_non_connected_components(G, source_id)

        #G = self.expand_graph_borders(G)

        # add important nodes that could be lost in previous filtering steps
        checked_nodes = []
        unchecked_nodes = [source_id]

        G, reachable_frontiers = self.reconnect_non_removable_nodes(G,
                                                                    checked_nodes,
                                                                    unchecked_nodes=unchecked_nodes,
                                                                    max_distance=self.border_threshold + 1.0)
        G = self.remove_non_connected_components(G, source_id)

        return G

    @staticmethod
    def extract_borders_from_graph(G, degree_tresh=9):
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
        border_ids = self.extract_borders_from_graph(G, degree_tresh=8)
        border_centroids = [self.centroids[idx] for idx in border_ids]
        #print("border_centroids:", len(border_centroids))

        # remove nodes from graph that are near to the borders
        # given a distance threshold
        if len(border_centroids) > 0:
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
        else:
            print("WARNING: border_centroids:", len(border_centroids))

        return G

    @staticmethod
    def remove_non_connected_components(G, source):
        """Remove all unconnected components not connected to the source node (position of the robot)
    
        :param G: 
        :param source: 
        :return: smaller G with the non connected components removed
        """
        try:
            conn_nodes = nx.node_connected_component(G, source)
            return G.subgraph(conn_nodes).copy()
        except Exception as e:
            traceback.print_exc()
            rospy.logwarn('Error returning connected components %s, continuing with G', e.message)
            return G

    def reconnect_non_removable_nodes(self, G, checked_nodes, unchecked_nodes=None, max_distance=0.1):
        """Add non removable nodes to the graph which can be deleted by
        previous filtering algorithms such as the source and destination points
        The checked_nodes list will be added to the graph after validation for a maximum distance established by the
        max_distance parameter. The unchecked_nodes will be added without previous validation, usefull for the start
        and end nodes.

        :param G: graph object
        :param checked_nodes: nodes to check against a maximum distance threshold
        :param unchecked_nodes:
        :param max_distance:
        :return: G with important nodes and edges added to it and nearest_checked_nodes, a list of the nearest
                checked nodes
        """

        nearest_checked_nodes = set()

        if not unchecked_nodes:
            unchecked_nodes = []

        # check if the source, the target, and the frontiers are reachable
        borderless_g_centroids = [tuple(self.centroids[v]) for v in sorted(G.nodes())]
        assert len(borderless_g_centroids) > 0, "The expand borders function did not leave any nodes, maybe" \
                                                "the treshold is too high? " \
                                                "border_tresh:{}".format(self.border_threshold)

        borderless_g_dict = {i: v for i, v in enumerate(sorted(G.nodes()))}
        borderless_kdtree = spatial.KDTree(borderless_g_centroids)

        # unchecked nodes
        for n_idx in unchecked_nodes:
            if G.has_node(n_idx):
                continue

            d, nearest_idx = borderless_kdtree.query(self.centroids[n_idx])
            nearest_idx = int(nearest_idx)
            G.add_node(n_idx)
            G.add_edge(n_idx, borderless_g_dict[nearest_idx])

        # checked nodes
        for n_idx in checked_nodes:
            if G.has_node(n_idx):
                nearest_checked_nodes.add(n_idx)
                continue

            d, nearest_idx = borderless_kdtree.query(self.centroids[n_idx])
            nearest_idx = int(nearest_idx)

            if d <= max_distance:
                G.add_node(n_idx)
                G.add_edge(n_idx, borderless_g_dict[nearest_idx])
                nearest_checked_nodes.add(n_idx)
            else:
                nearest_checked_nodes.add(nearest_idx)

        return G, nearest_checked_nodes

    def run_graph_process(self, graph_metric_type, source_id, target_frontiers, target_weights, return_dict, is_debug=False):
        """Generate a graph and run the path planning using a metric

        :param graph_metric_type: the metric type to use in this graph process
        :param source_id: source node id
        :param target_id: target node id
        :param return_dict: the shared variable to send and receive data from processes
        :param is_debug: a flag to define some debug parameters such as 3D plot of the connectivity graph
        :return: Nothing, it uses the return_dict variable for this
        """
        rospy.loginfo("Started graph process: %s", graph_metric_type.name)

        # graph creation and filtering
        start_time = time.process_time()
        G = self.create_graph_from_pcloud()
        #print("time create_graph_from_pcloud:", (time.process_time() - start_time))

        start_time = time.process_time()
        G = self.prepare_graph(G, source_id)
        #print("time prepare_graph:", (time.process_time() - start_time))

        if is_debug:
            self.plot_graph_3d(G,
                               title=graph_metric_type.name,
                               source_id=source_id)

        # dijkstra_results = []
        # for frontier_id in target_frontiers:
        #     g_search = graph_search.MeshGraphSearch(G,
        #                                             graph_metric_type,
        #                                             list(self.centroids),
        #                                             list(self.normals),
        #                                             c_short=self.shortest_comb_weight,
        #                                             c_energy=self.energy_comb_weight,
        #                                             c_traversal=self.transversality_comb_weight,
        #                                             pybullet_angle_client=None,
        #                                             optimization_angle_client=None)
        #     (length, path) = g_search.dijkstra_search({source_id}, frontier_id)
        #
        #     dijkstra_results.append({
        #         "path": path,
        #         "length": length,
        #         "frontier_id": frontier_id,
        #         "source_id": source_id,
        #         "p_cloud_size": len(self.centroids)
        #     })

        g_search = rrt_graph_search.RRTGraphSearch(G,
                                                   graph_metric_type,
                                                   list(self.centroids),
                                                   list(self.normals),
                                                   c_short=self.shortest_comb_weight,
                                                   c_energy=self.energy_comb_weight,
                                                   c_traversal=self.transversality_comb_weight)
        (length, path) = g_search.rrt_search(source_id, target_frontiers, target_weights, is_debug=is_debug)

        rospy.loginfo("[Planner] Ended process: %s %.2f seconds", graph_metric_type.name, g_search.last_execution_time)
        #rospy.loginfo("Ended process: %s length:%s path:%s", graph_metric_type.name, length, path)

        if path is not None:
            return_dict[graph_metric_type] = g_search
        else:
            # could not get any path for the pair source/target
            return_dict[graph_metric_type] = None

    def run(self, source_id, target_frontiers, target_weights, is_debug=False):
        """Run the graphs using the provided metrics and returns the path list
        :param is_debug:
        :param source_id: node source id
        :param target_id: node target id
        :return:
        """
        processes_list = []
        #manager = multiprocessing.Manager()
        #return_dict = manager.dict()
        return_dict = {}

        for gmt in self.graph_metrics_types:
            self.run_graph_process(gmt, source_id, target_frontiers, target_weights, return_dict, is_debug)

        # prepare result from face id to world point
        world_path_dict = {}
        for gmt in self.graph_metrics_types:
            p_finder = return_dict[gmt]
            metric_name = gmt.name

            if p_finder is None:
                rospy.logerr("Cannot find path for metric: %s", metric_name)
                continue

            # p_finder.print_path_metrics()

            p_list = [self.centroids[f_id] for f_id in p_finder.get_path()]
            world_path_dict[gmt] = {'face_path': p_finder.get_path(),
                                    'path': p_list,
                                    'cost': p_finder.get_path_distance(),
                                    'time': p_finder.last_execution_time,
                                    'target_idx': p_finder.target_idx}

        return world_path_dict

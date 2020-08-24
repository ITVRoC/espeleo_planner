#!/usr/bin/env python

import rospy
import pymesh
import networkx as nx
import multiprocessing
import graph_search
import numpy as np
import time
from scipy import spatial


class MeshPathFinder:
    """
    Mesh Path Finder given a mesh, a list of metrics and a source and destination points
    calculates the optimum paths
    """

    def __init__(self, mesh_path, graph_metrics):
        """
        Mesh Path Finder constructor
        :param mesh_path: path to mesh .stl file
        :param graph_metrics: list of path metrics to calculate (GraphMetric object)
        """

        self.mesh_path = mesh_path
        self.mesh = pymesh.load_mesh(self.mesh_path)

        if isinstance(graph_metrics, (list, tuple)):
            self.graph_metrics = graph_metrics
        elif graph_metrics is not None:
            self.graph_metrics = [self.graph_metrics]
        else:
            raise TypeError("graph_metrics is not a valid object type [list, tuple]")

        self.transversality_tresh = 35  # max inclination (in degrees) the robot could climb
        self.bumpiness_tresh = 0.5  # maximum bump the robot could jump between surfaces TODO add reference here
        self.border_tresh = 0.3  # distance to expand from borders to other face centroids

        self.shortest_comb_weight = 0.25  # this is a shortest weight to combine the weights of the metrics
        self.energy_comb_weight = 0.25  # this is a energy weight to combine the weights of the metrics
        self.transversality_comb_weight = 0.50  # this is a transversality weight to combine the weights of the metrics

        self.mesh.enable_connectivity()                 # enables connectivity on mesh
        self.mesh.add_attribute("face_centroid")        # adds the face centroids to be accessed
        self.mesh.add_attribute("face_normal")          # adds the face normals to be accessed

        self.faces = self.mesh.faces
        self.centroids = self.mesh.get_face_attribute("face_centroid")
        self.normals = self.mesh.get_face_attribute("face_normal")
        self.frontiers = set()

        rospy.loginfo("Vertex and Face count: %d, %d" % (self.mesh.num_vertices, self.mesh.num_faces))
        rospy.loginfo("Dimensions and Vertexes in a face: %d, %d" % (self.mesh.dim, self.mesh.vertex_per_face))

    def extract_frontiers(self):
        """

        :return:
        """
        for face_id in range(0, self.mesh.num_faces):
            adj_faces = self.mesh.get_face_adjacent_faces(face_id)
            if len(adj_faces) <= 2:
                self.frontiers.add(face_id)

        return self.frontiers

    def create_graph_from_mesh(self):
        """Create a graph from the mesh's faces centroids conecting nodes using the conectivity graph of the
        original mesh
        :param G:
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

    def filter_graph_by_traversable_faces(self, G):
        """Remove non traversable faces from the graph
        CAUTION: this can return a non fully connected graph with multiple
        smaller subgraphs

        :param G:
        :return: a graph with only traversable faces
        """
        for face_idx in list(G.nodes()):
            face_inclination = graph_search.MeshGraphSearch.calculate_traversal_angle(self.normals[face_idx])
            if face_inclination > self.transversality_tresh:
                G.remove_node(face_idx)

        return G

    def expand_graph_borders(self, G, source, target):
        """Remove nodes from a graph that are withing a distance treshold from the borders
        this helps to generate routes where the robot could actually move (narrow passages) and prevent
        collisions with obstacles and "static falls"

        :param G:
        :return: a smaller graph G' with the expanded borders removed
        """
        # estimate borders of the remainder graph
        # border_nodes = [Gconn.info(v) for v in sorted(Gconn.nodes())]
        border_centroids = []
        for v in sorted(G.nodes()):
            if nx.degree(G, v) <= 9:
                border_centroids.append(tuple(self.centroids[v]))  # tuples are hashable! lists are not

        # remove nodes from graph that are near to the borders
        # given a distance treshold
        border_kdtree = spatial.KDTree(border_centroids)
        for v in list(G.nodes()):
            point = self.centroids[v]
            distances, nearest_idx = border_kdtree.query([point])
            obstacle_d = distances[0]
            if obstacle_d <= self.border_tresh:
                G.remove_node(v)

        # remove small connected components
        for component in list(nx.connected_components(G)):
            if len(component) < 3:
                for node in component:
                    G.remove_node(node)

        # check if the source, the target, and the frontiers are reachable
        # TODO: add frontiers here too
        non_removable_nodes = [source, target]
        if not all(G.has_node(n_idx) for n_idx in non_removable_nodes):
            borderless_g_centroids = [tuple(self.centroids[v]) for v in sorted(G.nodes())]
            assert len(borderless_g_centroids) > 0, "The expand borders function did not leave any nodes, maybe the " \
                                                    "treshold is too high? border_tresh:{}".format(self.border_tresh)

            borderless_g_dict = {i: v for i, v in enumerate(sorted(G.nodes()))}
            borderless_kdtree = spatial.KDTree(borderless_g_centroids)

            for n_idx in non_removable_nodes:
                if G.has_node(n_idx):
                    continue

                distances, nearest_idx = borderless_kdtree.query(self.centroids[n_idx])
                print "n_idx:", n_idx, "distances:", distances, "nearest_idx:", nearest_idx, "non_removable_nodes:", non_removable_nodes

                G.add_node(n_idx)
                G.add_edge(n_idx, borderless_g_dict[nearest_idx])

        return G

    def run_graph_process(self, graph_metric, return_dict, is_debug=True):
        """Generate a graph and run the path planning using a metric

        :param graph_metric: the metric to use in this graph process
        :param return_dict: the shared variable to send and receive data from processes
        :param is_debug: a flag to define some debug parameters such as 3D plot of the conectivity graph
        :return: Nothing, it uses the return_dict variable for this
        """
        rospy.loginfo("Started graph process: %s", graph_metric.get_metric().name)
        start_time = time.clock()

        # graph creation and filtering
        G = self.create_graph_from_mesh()
        G = self.filter_graph_by_traversable_faces(G)
        G = self.expand_graph_borders(G, graph_metric.get_source(), graph_metric.get_target())

        g_search = graph_search.MeshGraphSearch(G,
                                                graph_metric.get_metric(),
                                                self.centroids,
                                                self.normals,
                                                c_short=self.shortest_comb_weight,
                                                c_energy=self.energy_comb_weight,
                                                c_traversal=self.transversality_comb_weight)

        if is_debug:
            g_search.plot_graph_3d()

        (length, path) = g_search.dijkstra_search({graph_metric.get_source()}, graph_metric.get_target())
        rospy.loginfo("Ended process: %s %.2f seconds", graph_metric.get_metric().name, time.clock() - start_time)

        if path is not None:
            return_dict[graph_metric.get_metric()] = g_search
        else:
            # could not get any path for the pair source/target
            return_dict[graph_metric.get_metric()] = None

    def run(self, is_multithread=False):
        """Run the graphs using the provided metrics and returns the path list
        :param is_multithread: a flag to define if the processes are going to be executed in threads (processes) or not
        :return:
        """

        processes_list = []
        manager = multiprocessing.Manager()
        return_dict = manager.dict()

        if is_multithread:
            # run the processes simultaneously
            for gm in self.graph_metrics:
                graph_proc = multiprocessing.Process(target=self.run_graph_process, args=[gm, return_dict])
                graph_proc.start()
                processes_list.append(graph_proc)

            for process in processes_list:
                process.join()
        else:
            # run processes sequentially
            for gm in self.graph_metrics:
                self.run_graph_process(gm, return_dict)

        # prepare result from face id to world point
        world_path_dict = {}
        for gm in self.graph_metrics:
            p_finder = return_dict[gm.get_metric()]
            metric_name = gm.get_metric().name

            if p_finder is None:
                rospy.logerr("Cannot find path for metric: %s", metric_name)
                continue

            p_finder.print_path_metrics()

            p_list = [self.centroids[f_id] for f_id in p_finder.get_path()]
            world_path_dict[gm.get_metric()] = p_list

        return world_path_dict

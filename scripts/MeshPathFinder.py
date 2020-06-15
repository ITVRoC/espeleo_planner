# Created By Frederico Sousa
# MeshPathFinder API, find paths in .stl mesh based on metrics
# Useful - To Debug the code - run only one metric to debug it or use PyCharm "Attach to process" functionality


# Improvements and bugs to solve
# Catch the node not found exception from networkx and rerun code free
# Add transversality bound and number of neighbours in user GUI

import pymesh
import pybullet as p
import networkx as nx
import numpy as np
import math
from prettytable import PrettyTable
from tqdm import tqdm
from multiprocessing import Process
import vtk
from vtk import vtkInteractorStyleTrackballCamera
import sys
import os


class MeshPathFinder:
    def __init__(self, mesh_path, graph_metrics, robot_urdf=None, map_urdf=None, client_mode=None):
        self.mesh_path = mesh_path                          # path to mesh .stl file
        self.mesh_ref = pymesh.load_mesh(self.mesh_path)    # the logical reference of the mesh loaded on library
        self.graphs_metrics = graph_metrics                 # the tuple of metrics to be run
        self.mesh_ref.enable_connectivity()                 # enables connectivity on mesh
        self.mesh_ref.add_attribute("face_centroid")        # adds the face centroids to be accessed
        self.mesh_ref.add_attribute("face_normal")          # adds the face normals to be accessed
        self.faces = self.mesh_ref.faces                    # list containing all the faces on mesh
        self.centroids = self.mesh_ref.get_face_attribute("face_centroid")      # list containing all mesh centroids
        self.normals = self.mesh_ref.get_face_attribute("face_normal")          # list containing all mesh normals
        self.transversality_bound = 30          # this is a transversality bound to set a max degree of transversality
        self.shortest_comb_weight = 0.25        # this is a shortest weight to combine the weights of the metrics
        self.energy_comb_weight = 0.25          # this is a energy weight to combine the weights of the metrics
        self.transversality_comb_weight = 0.50  # this is a transversality weight to combine the weights of the metrics

        if (robot_urdf and map_urdf) is not None:

            self.robot_urdf = robot_urdf
            self.map_urdf = map_urdf
            self.map_id = None
            self.robot_id = None

            if client_mode == "GUI":
                p.connect(p.GUI)
                self.bullet_client = p
            elif client_mode == "DIRECT":
                p.connect(p.DIRECT)
                self.bullet_client = p
            else:
                print("MESH PATH FIND ERROR! "
                      "If the URDF's are set, a robot urdf, map urdf and a client mode are needed.")
                print("Client Mode: GUI or DIRECT")
                sys.exit(1)
        else:
            self.bullet_client = None
            self.robot_urdf = None
            self.map_urdf = None

    def set_shortest_weight(self, shortest_weight):
        self.shortest_comb_weight = shortest_weight

    def set_transversality_weight(self, transversal_weight):
        self.transversality_comb_weight = transversal_weight

    def set_energy_weight(self, energy_weight):
        self.energy_comb_weight = energy_weight

    def combined_weights_from_gui(self, shortest_w, energy_w, transversal_w):
        self.shortest_comb_weight = shortest_w
        self.energy_comb_weight = energy_w
        self.transversality_comb_weight = transversal_w

    def set_bullet_client(self):
        self.bullet_client.setGravity(0, 0, -10)

        map_start_pos = [0, 0, 0]
        map_start_orientation = self.bullet_client.getQuaternionFromEuler([-math.pi / 2, 0, 0])
        self.map_id = self.bullet_client.loadURDF(self.map_urdf, map_start_pos, map_start_orientation)

        robot_start_pos = [0, 0, 0]
        robot_start_orientation = self.bullet_client.getQuaternionFromEuler([-math.pi / 2, 0, 0])
        self.robot_id = self.bullet_client.loadURDF(self.robot_urdf, robot_start_pos, robot_start_orientation,
                              flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)

        # Define friction
        lf = 1
        sf = 1
        rf = 0
        cf = -1
        ld = 1
        ad = 1
        cd = -1

        self.bullet_client.changeDynamics(self.robot_id, 0, lateralFriction=lf, spinningFriction=sf, rollingFriction=rf,
                                          linearDamping=ld, angularDamping=ad, contactStiffness=cf)
        self.bullet_client.changeDynamics(self.robot_id, 1, lateralFriction=lf, spinningFriction=sf, rollingFriction=rf,
                                          linearDamping=ld, angularDamping=ad, contactStiffness=cf)
        self.bullet_client.changeDynamics(self.robot_id, 2, lateralFriction=lf, spinningFriction=sf, rollingFriction=rf,
                                          linearDamping=ld, angularDamping=ad, contactStiffness=cf)
        self.bullet_client.changeDynamics(self.robot_id, 3, lateralFriction=lf, spinningFriction=sf, rollingFriction=rf,
                                          linearDamping=ld, angularDamping=ad, contactStiffness=cf)
        self.bullet_client.changeDynamics(self.robot_id, 4, lateralFriction=lf, spinningFriction=sf, rollingFriction=rf,
                                          linearDamping=ld, angularDamping=ad, contactStiffness=cf)
        self.bullet_client.changeDynamics(self.robot_id, 5, lateralFriction=lf, spinningFriction=sf, rollingFriction=rf,
                                          linearDamping=ld, angularDamping=ad, contactStiffness=cf)
        self.bullet_client.changeDynamics(self.map_id, -1, lateralFriction=lf, spinningFriction=sf, rollingFriction=rf,
                                          linearDamping=ld, angularDamping=ad, contactStiffness=cf, contactDamping=cd)

    @staticmethod
    def path_to_file(path, name):
        path_file = open("TxtPaths/dijkstraPath" + name + ".txt", "w")
        for i in path:
            path_file.write(str(i) + "\n")
        path_file.close()

    @staticmethod
    def file_to_list(dijkstra_file_path):
        dijkstra_list = []
        dijkstra_file = open(dijkstra_file_path, "r")
        i = 0
        for line in dijkstra_file:
            form_line = line.split("\n")
            number = int(form_line[0])
            dijkstra_list.append(number)
            i += 1

        dijkstra_file.close()
        return dijkstra_list

    @staticmethod
    def print_matrix_adjacency(g):
        m_a = nx.adjacency_matrix(g)
        print(m_a)

    def mesh_properties(self):
        print("Vertex and Face count: %d, %d" % (self.mesh_ref.num_vertices, self.mesh_ref.num_faces))
        print("Dimensions and Vertexes in a face: %d, %d" % (self.mesh_ref.dim, self.mesh_ref.vertex_per_face))

    def set_transversality_bound(self, transv_value):   # edit the bound of transversality of faces to be excluded
        self.transversality_bound = transv_value        # transv_value in degrees

    def deviation(self, path_list, mean_dist, mean_ang, mean_ener, mean_rot):
        deviation_dist = 0
        deviation_angles = 0
        deviation_energy = 0
        deviation_rot = 0

        for i in range(0, len(path_list) - 1):
            node_source = path_list[i]
            node_target = path_list[i + 1]

            dist = self.euclidian_distance_weight(node_source, node_target)
            ang = self.terrain_weight(node_source)

            if node_source == path_list[0]:
                predecessor = -1
            else:
                predecessor = path_list[i - 1]

            rot = self.calc_rotation(node_source, node_target, predecessor, path_list[-1])
            enrg = self.energy_weight(node_source, node_target, predecessor, path_list[-1])

            deviation_dist += np.sqrt(math.pow((dist - mean_dist), 2) / len(path_list))
            deviation_angles += np.sqrt(math.pow((ang - mean_ang), 2) / len(path_list))
            deviation_energy += np.sqrt(math.pow((enrg - mean_ener), 2) / len(path_list))
            deviation_rot += np.sqrt(math.pow((rot - mean_rot), 2) / len(path_list))

        return deviation_dist, deviation_energy, deviation_rot, deviation_angles

    def calc_rotation(self, node_source, node_target, predecessor, final):
        points_current = self.centroids[node_source]  # points [x, y, z] of the current node
        points_target = self.centroids[node_target]  # points [x, y, z] of the target node

        if predecessor == -1:
            points_predecessor = self.centroids[final]
            vector_p2c = [points_predecessor[0] - points_current[0],
                         points_predecessor[1] - points_current[1]]  # vector between the predecessor and current node

        else:
            points_predecessor = self.centroids[predecessor]  # points [x, y, z] of the predecessor node
            vector_p2c = [points_current[0] - points_predecessor[0], points_current[1] - points_predecessor[1]]

        vector_c2t = [points_target[0] - points_current[0],
                     points_target[1] - points_current[1]]  # vector between the current and target node

        rotation = self.angle_2_vectors(vector_p2c, vector_c2t)
        return rotation

    def print_path_properties(self, path_list, name):
        table = PrettyTable()
        distance = 0
        big_distance = 0
        minor_distance = 1000000
        angles = 0
        big_angle = 0
        minor_angle = 10000000
        rotation = 0
        big_rotation = 0
        minor_rotation = 1000000
        energy = 0
        big_energy = 0
        minor_energy = 10000000
        deviation_list = []

        for i in range(0, (len(path_list) - 1)):
            node_source = path_list[i]
            node_target = path_list[i + 1]

            if node_source == path_list[0]:
                predecessor = -1
            else:
                predecessor = path_list[i - 1]

            dist = self.euclidian_distance_weight(node_source, node_target)
            distance += dist

            ang = self.terrain_weight(node_source)
            angles += ang

            rot = self.calc_rotation(node_source, node_target, predecessor, path_list[-1])
            rotation += rot

            enrg = self.energy_weight(node_source, node_target, predecessor, path_list[-1])
            energy += enrg

            if ang >= big_angle:
                big_angle = ang
            elif ang <= minor_angle:
                minor_angle = ang

            if dist >= big_distance:
                big_distance = dist
            elif dist <= minor_distance:
                minor_distance = dist

            if rot >= big_rotation:
                big_rotation = rot
            elif rot <= minor_rotation:
                minor_rotation = rot

            if enrg >= big_energy:
                big_energy = enrg
            elif enrg <= minor_energy:
                minor_energy = enrg

        mean_rot = rotation / len(path_list)
        mean_dist = distance / len(path_list)
        mean_ener = energy / len(path_list)
        mean_angle = angles / len(path_list)

        deviation_list = self.deviation(path_list, mean_dist, mean_angle, mean_ener, mean_rot)


        print("\n")
        print("\t\t\t\t\t\t\tPATH PROPERTIES %s:" % name)
        table.field_names = ["DESCRIPTION", "DISTANCE", "ENERGY", "ROTATIONS", "TRANSVERSALITY"]
        table.add_row(["MINOR", minor_distance, minor_energy, minor_rotation, minor_angle])
        table.add_row(["BIGGER", big_distance, big_energy, big_rotation, big_angle])
        table.add_row(["MEAN", mean_dist, mean_ener, mean_rot, mean_angle])
        table.add_row(["DEVIATION", deviation_list[0], deviation_list[1], deviation_list[2], deviation_list[3]])
        table.add_row(["TOTAL", distance, energy, rotation, angles])
        print(table)

    def save_path_points_in_file(self, path_list, filename):
        points_file = open(filename, "w")
        for face in path_list:
            points = self.centroids[face]
            x = np.float128(points[0])
            y = np.float128(points[1])
            z = np.float128(points[2])
            points_file.write(str(x) + "," + str(y) + "," + str(z) + "\n")
        points_file.close()

    def adjacency_points_file(self, node, filename):
        adjacency_points = open(filename, "w")

        adjacents = []
        vertices = self.faces[node]

        for vertice in vertices:
            ad = self.mesh_ref.get_vertex_adjacent_faces(vertice)
            for face in ad:
                if face == node:
                    pass
                else:
                    adjacents.append(face)

        adjacents = list(dict.fromkeys(adjacents))
        points = self.centroids[node]
        x = np.float128(points[0])
        y = np.float128(points[1])
        z = np.float128(points[2])
        adjacency_points.write(str(x) + " , " + str(y) + " , " + str(z) + "\n")
        for face in adjacents:
            points = self.centroids[face]
            x = np.float128(points[0])
            y = np.float128(points[1])
            z = np.float128(points[2])
            adjacency_points.write(str(x) + " , " + str(y) + " , " + str(z) + "\n")
        adjacency_points.close()

    def euclidian_distance_weight(self, node_source, node_target):
        # calculates the euclidian distance between two nodes, based on its centroids[x,y,z]
        source_points = self.centroids[node_source]
        target_points = self.centroids[node_target]
        delta_x = math.pow((np.float128(source_points[0]) - np.float128(target_points[0])), 2)
        delta_y = math.pow((np.float128(source_points[1]) - np.float128(target_points[1])), 2)
        delta_z = math.pow((np.float128(source_points[2]) - np.float128(target_points[2])), 2)
        sum_dist = delta_x + delta_y + delta_z
        sum_dist = np.float128(sum_dist)
        length = math.sqrt(sum_dist)
        length = np.float128(length)
        return length

    def angle_two_faces(self, node_source, node_target):  # Returns the angle between two faces of the mesh
        source_normal_p = self.normals[node_source]
        target_normal_p = self.normals[node_target]
        dp = (source_normal_p[0] * target_normal_p[0]) + (source_normal_p[1] * target_normal_p[1]) + (
                    source_normal_p[2] * target_normal_p[2])
        radians = math.acos(dp)
        angle_degrees = radians * (180 / math.pi)
        return angle_degrees

    def angle_2_vectors(self, vector1, vector2):
        scalar = np.dot(vector1, vector2)  # scalar product between the face normal and Z axe
        norms = np.linalg.norm(vector1) * np.linalg.norm(vector2)  # norm between face normal and Z Axe
        if norms == 0:
            cos_value = 0
        else:
            cos_value = scalar / norms

        if cos_value > 1:  # treating floating point problems
            cos_value = 1

        elif cos_value < -1:  # treating floating point problems
            cos_value = -1

        radians = math.acos(cos_value)  # arccos of scalar/norms
        degrees = math.degrees(radians)
        return degrees

    def terrain_weight(self, node_target):
        source_normal_p = self.normals[node_target]  # face source normal
        z_axe = [0, 0, -1]  # vector representing the Z axe
        scalar = np.dot(source_normal_p, z_axe)  # scalar product between the face normal and Z axe
        norms = np.linalg.norm(source_normal_p) * np.linalg.norm(z_axe)  # norm between face normal and Z Axe
        cos_angle = scalar / norms
        radians = math.acos(cos_angle)  # arccos of scalar/norms
        angle_degrees = math.degrees(radians)
        angle_degrees = math.fabs(angle_degrees)
        return angle_degrees

    def angle_rotation(self, source_point, target_point):
        radians = math.atan2((target_point[1] - source_point[1]), (target_point[0] - source_point[0]))
        angle = math.degrees(radians)
        angle = math.fabs(angle)
        rot = (angle * 2 * math.pi * 0.210) / 360
        return rot

    def energy_weight_non_rot(self, node_source, node_target):
        u = 1
        m = 20  # kg
        g = 9.8  # m/s^2
        points_source = self.centroids[node_source]
        points_target = self.centroids[node_target]
        vector2_faces = [points_target[0] - points_source[0], points_target[1] - points_source[1],
                        points_target[2] - points_source[2]]
        vector_xy = [0, 0, -1]
        angle = self.angle_2_vectors(vector2_faces, vector_xy)
        dist = self.euclidian_distance_weight(node_source, node_target)
        energy = math.fabs((u * m * g * math.cos(angle)) + (m * g * math.sin(angle))) * dist
        return energy

    def energy_weight(self, node_source, node_target, predecessor, final):
        points_source = self.centroids[node_source]  # points of the center of the source face (node)
        points_target = self.centroids[node_target]  # points of the center of the target face (node)

        vector2_faces = [points_target[0] - points_source[0], points_target[1] - points_source[1],
                        points_target[2] - points_source[2]]  # vector between the the face and target points

        if predecessor == -1:
            points_predecessor = self.centroids[final]
            vector_p2c = [points_predecessor[0] - points_source[0],
                         points_predecessor[1] - points_source[1]]  # vector between the predecessor and current node

        else:
            points_predecessor = self.centroids[predecessor]  # points [x, y, z] of the predecessor node
            vector_p2c = [points_source[0] - points_predecessor[0], points_source[1] - points_predecessor[1]]

        vector_c2t = [points_target[0] - points_source[0],
                     points_target[1] - points_source[1]]  # vector between the current and target node

        rot = self.angle_2_vectors(vector_p2c, vector_c2t)  # returns the minor angle between the two vectors,
        # representing the rotation between the two faces

        vector_xy = [0, 0, -1]  # normal of the XY plan = Z Axe
        angle = self.angle_2_vectors(vector2_faces, vector_xy)  # gets the angle between the two vectors
        angle = 90 - angle
        dist = self.euclidian_distance_weight(node_source, node_target)  # distance between the faces (nodes)

        if angle < 0:
            energy = (((37735.9 * rot) / 360) + ((-475.07 * angle) + 1089.3)) * dist
        else:
            energy = (((37735.9 * rot) / 360) + ((564.97 * angle) + 1364.9)) * dist

        return energy

    def energy_weight_min_max(self, node_source, node_target):
        points_source = self.centroids[node_source]  # points of the center of the source face (node)
        points_target = self.centroids[node_target]  # points of the center of the target face (node)
        vector2_faces = [points_target[0] - points_source[0], points_target[1] - points_source[1],
                        points_target[2] - points_source[2]]  # vector between the the face and target points
        vector_xy = [0, 0, -1]  # normal of the XY plan = Z Axe
        angle = self.angle_2_vectors(vector2_faces, vector_xy)  # gets the angle between the two vectors
        angle = 90 - angle
        dist = self.euclidian_distance_weight(node_source, node_target)  # distance between the faces (nodes)
        rot = self.angle_rotation(points_source, points_target)  # rotation between the two faces
        if angle < 0:
            energy = (((37735.9 * rot) / 360) + ((-475.07 * angle) + 1089.3)) * dist
        else:
            energy = (((37735.9 * rot) / 360) + ((564.97 * angle) + 1364.9)) * dist

        return energy

    def get_face_adjacency(self, face_vertices, node_current):
        adjacents = []
        for vertice in face_vertices:
            vertex_adjancency = self.mesh_ref.get_vertex_adjacent_faces(vertice)
            for faceAdjacent in vertex_adjancency:
                if faceAdjacent == node_current:
                    pass
                else:
                    adjacents.append(faceAdjacent)

        adjacents = list(dict.fromkeys(adjacents))
        return adjacents

    def find_min_max_global(self):  # mesh pointer, faces list with its vertices, centroids list and
        print("\nFinding Min Max List...")
        max_dist = 0  # normals list
        min_dist = 100000000
        max_rot = 0
        min_rot = 180
        min_tra = 100000000
        max_tra = 0
        min_energy = 10000000
        max_energy = 0

        with tqdm(total=self.mesh_ref.num_faces) as pbar:  # tqdm for progress bar
            for node_index in range(0, self.mesh_ref.num_faces):
                face = self.faces[node_index]
                adjacencies = self.get_face_adjacency(face, node_index)
                for i in adjacencies:

                    tra = self.terrain_weight(i)
                    energ = self.energy_weight_min_max(node_index, i)

                    if energ >= (min_energy * 5):
                        pass

                    else:
                        if tra >= max_tra:
                            max_tra = tra
                        elif tra <= min_tra:
                            min_tra = tra

                        dist = self.euclidian_distance_weight(node_index, i)

                        if dist >= max_dist:
                            max_dist = dist
                        elif dist <= min_dist:
                            min_dist = dist

                        if energ >= max_energy:
                            max_energy = energ
                        elif energ <= min_energy:
                            min_energy = energ
                pbar.update(1)

        return max_dist, min_dist, max_rot, min_rot, max_tra, min_tra, max_energy, min_energy

    @staticmethod
    def normalize_length(length, max_dist, min_dist):
        np.float128(length)
        norma_length_data = ((length - min_dist) / (max_dist - min_dist)) * ((10 - 1) + 1)
        return norma_length_data

    @staticmethod
    def normalize_rotation(rotation, max_rot, min_rot):
        norma_rot_data = ((rotation - min_rot) / (max_rot - min_rot)) * ((10 - 1) + 1)
        return norma_rot_data

    @staticmethod
    def normalize_transversal(transversal, max_tra, min_tra):
        if transversal == 0:
            return 1

        normal_tra_data = ((transversal - min_tra) / (max_tra - min_tra)) * ((10 - 1) + 1)
        return normal_tra_data

    @staticmethod
    def normalize_energy(energy, max_energy, min_energy):
        normal_energ_data = ((energy - min_energy) / (max_energy - min_energy)) * ((10 - 1) + 1)
        return normal_energ_data

    def edge_weight(self, node_source, node_target, min_max_list, predecessor, final, metric):

        if metric == 1:
            length = self.euclidian_distance_weight(node_source, node_target)  # calc euclidian dist between 2 nodes
            return length

        if metric == 2:
            length = self.euclidian_distance_weight(node_source, node_target)
            transversal = self.terrain_weight(node_target)
            return transversal + length

        if metric == 3:
            if predecessor == -1:
                predecessor = final
            energy_spent = self.energy_weight(node_source, node_target, predecessor, final)
            return energy_spent

        if metric == 4:
            length = self.euclidian_distance_weight(node_source, node_target)  # calc euclidian dist between two nodes
            length_normal = self.normalize_length(length, min_max_list[0], min_max_list[1])
            transversal = self.terrain_weight(node_target)  # calculate angle between Z Axe and face normal
            transversal_normal = self.normalize_transversal(transversal, min_max_list[4], min_max_list[5])
            energy_spent = self.energy_weight(node_source, node_target, predecessor, final)
            energy_normal = self.normalize_energy(energy_spent, min_max_list[6], min_max_list[7])

            rotation = self.calc_rotation(node_source, node_target, predecessor, final)
            weight_comb = ((self.shortest_comb_weight * length_normal) + rotation) + \
                          (self.energy_comb_weight * transversal_normal) + (self.energy_comb_weight * energy_normal)
            return weight_comb

        if metric == 5:
            rotation = self.calc_rotation(node_source, node_target, predecessor, final)
            return rotation

    def create_graph(self, graph, metric, source, target):  # &networkx Graph, &mesh reference, &centroids face list
        adjacency_list = []  # &normals face list, metric which weights will be
        faces_excluded = []

        print("\nCreating Graph...")
        with tqdm(total=self.mesh_ref.num_faces) as pbar:  # tqdm for progress bar
            for face_index in range(0, self.mesh_ref.num_faces):
                # run through all the faces of mesh and adds them as nodes
                transversal = self.terrain_weight(face_index)
                face = self.faces[face_index]
                adjacents = []

                for vertice in face:
                    vertex_adjacency = self.mesh_ref.get_vertex_adjacent_faces(vertice)
                    for face_adjacent in vertex_adjacency:
                        if face_adjacent == face_index:
                            pass
                        else:
                            adjacents.append(face_adjacent)

                adjacents = list(dict.fromkeys(adjacents))
                adjacency_list.append(adjacents)

                if metric >= 1 and (len(adjacency_list[face_index]) <= 10 or transversal >= self.transversality_bound)\
                        and (face_index != source or face_index != target):
                    # if node has less than 9 edges or angle (between normal
                    faces_excluded.append(
                        face_index)  # and Z Axe) bigger than 30 degrees, the node is isolated so it wont be considered

                else:
                    graph.add_node(face_index)  # add the nodes based on the mesh faces

                # graph.add_node(face_index)
                pbar.update(1)  # progress bar is updated

        faces_excluded = list(dict.fromkeys(faces_excluded))  # remove duplicates in list
        zipb_obj = zip(faces_excluded, faces_excluded)
        dict_excluded = dict(zipb_obj)  # creates a dictionary of the excluded faces

        print("Adding Edges...")

        with tqdm(total=self.mesh_ref.num_faces) as pbar:  # tqdm for progress bar
            for i in range(0, self.mesh_ref.num_faces):  # loops through the faces, i is the face (node) source

                if i in dict_excluded:  # if face was excluded, continue the execution to the next face
                    continue

                adjacents = adjacency_list[i]
                for j in range(0, len(adjacents)):  # gets all the faces adjacent to a (i) face
                    edge_add = adjacents[j]

                    if edge_add in dict_excluded:  # if face is excluded, make sure to not connect with it
                        continue

                    else:
                        graph.add_edge(i, edge_add, weight=1)  # adds an edge between i and edgeAdd

                pbar.update(1)

    def run_graph_process(self, metric, source, target, name):  # method to run parallel processes
        # graph_props = list[metric, source, target, name]
        graph = nx.Graph()  # instantiate a networkX graph object
        graph.name = name  # sets a graph name
        self.create_graph(graph, metric, source, target)  # create a graph based on its metric
        if metric == 4:     # metric 4 ros::uses a global min max list with estimated values of min max
            min_max_list = self.find_min_max_global()
        else:
            min_max_list = np.zeros(shape=8)

        if (self.bullet_client and self.robot_urdf and self.map_urdf) is not None:
            self.set_bullet_client()        # starts the pybullet client
            dijkstra_from_graph = nx.dijkstra_path(graph, source, target, self.centroids, self.normals,
                                                   min_max_list, self.shortest_comb_weight, self.energy_comb_weight,
                                                   self.transversality_comb_weight,
                                                   metric, self.robot_id, self.map_id, self.bullet_client)
            # find the dijkstra path with pybullet
        else:
            dijkstra_from_graph = nx.dijkstra_path(graph, source, target, self.centroids, self.normals, min_max_list,
                                             self.shortest_comb_weight, self.energy_comb_weight,
                                                   self.transversality_comb_weight, metric)  # find the dijkstra path
        self.path_to_file(dijkstra_from_graph, name)  # save the path to file
        self.save_path_points_in_file(dijkstra_from_graph, "TxtPaths/DijkstraPoints" + name + ".txt")
        # save the points from each node in the path

    @staticmethod
    def read_points_file(filename):
        file = open(filename, "r")
        point_list = []
        for line in file:
            points = line.split(",")
            x = float(points[0])
            y = float(points[1])
            z = float(points[2])
            point_list.append([np.float128(x), np.float128(y), np.float128(z)])

        file.close()
        return point_list

    @staticmethod
    def line_on_mesh(vtkPointsObject, nPoints,
                     color):  # returns an actor representing the line based in its vtkPointsObject

        color_val = [0.0, 0.0, 0.0]

        if color == "red":
            color_val = [1.0, 0.0, 0.0]
        elif color == "green":
            color_val = [0.0, 1.0, 0.0]
        elif color == "blue":
            color_val = [0.0, 0.0, 1.0]
        elif color == "pink":
            color_val = [1.0, 0.8, 0.9]
        elif color == "white":
            color_val = [1.0, 1.0, 1.0]
        elif color == "black":
            color_val = [0, 0, 0]

        # Create a spline and add the points
        spline = vtk.vtkParametricSpline()
        spline.SetPoints(vtkPointsObject)
        function_source = vtk.vtkParametricFunctionSource()
        function_source.SetUResolution(nPoints)
        function_source.SetParametricFunction(spline)

        # Map the spline
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(function_source.GetOutputPort())
        # Define the line actor
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(color_val)
        actor.GetProperty().SetLineWidth(5)
        return actor  # returns an actor to be added in the vtkRenderer

    @staticmethod
    def point_to_point(p1, p2, color):
        color_val = [0.0, 0.0, 0.0]

        if color == "red":
            color_val = [1.0, 0.0, 0.0]
        elif color == "green":
            color_val = [0.0, 1.0, 0.0]
        elif color == "blue":
            color_val = [0.0, 0.0, 1.0]
        elif color == "pink":
            color_val = [1.0, 0.8, 0.9]
        elif color == "white":
            color_val = [1.0, 1.0, 1.0]
        elif color == "black":
            color_val = [0, 0, 0]

        points = vtk.vtkPoints()
        points.InsertNextPoint(p1)
        points.InsertNextPoint(p2)

        # Create a spline and add the points
        spline = vtk.vtkParametricSpline()
        spline.SetPoints(points)
        function_source = vtk.vtkParametricFunctionSource()
        function_source.SetUResolution(2)
        function_source.SetParametricFunction(spline)

        # Map the spline
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(function_source.GetOutputPort())
        # Define the line actor
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(color_val)
        actor.GetProperty().SetLineWidth(5)
        return actor  # returns a actor to be added in the vtkRenderer

    def line_actor(self, filename, color):
        path_points = self.read_points_file(filename)

        points = vtk.vtkPoints()  # instantiating the vtkPointsObject
        for p in path_points:  # this loop inserts XYZ received from file in vtkPointsObject
            points.InsertNextPoint(p)  # insert the file points in vtkPointsObject

        actorLine = self.line_on_mesh(points, len(path_points), color)
        return actorLine

    def plot_path(self, path_colors):

        reader = vtk.vtkSTLReader()  # instantiate a stl reader from vtk
        reader.SetFileName(str(self.mesh_path))  # reads stl file from filepath
        reader.Update()

        # Add the grid points to a polydata object
        input_poly_data = vtk.vtkPolyData()
        input_poly_data = reader.GetOutput()

        bounds = 6 * [0.0]
        input_poly_data.GetBounds(bounds)

        # Find min and max z
        minz = bounds[4]
        maxz = bounds[5]

        # Create the color map
        color_lookup_table = vtk.vtkLookupTable()
        color_lookup_table.SetTableRange(minz, maxz)
        color_lookup_table.Build()

        # Generate the colors for each point based on the color map
        colors = vtk.vtkUnsignedCharArray()
        colors.SetNumberOfComponents(3)
        colors.SetName("Colors")

        for i in range(0, input_poly_data.GetNumberOfPoints()):
            # this loop defines the colors of the mesh based on the ZAxe
            p = 3 * [0.0]
            input_poly_data.GetPoint(i, p)

            dcolor = 3 * [0.0]
            color_lookup_table.GetColor(p[2], dcolor)

            color = 3 * [0.0]
            for j in range(0, 3):
                color[j] = int(255.0 * dcolor[j])

            try:
                colors.InsertNextTupleValue(color)
            except AttributeError:
                # For compatibility with new VTK generic data arrays.
                colors.InsertNextTypedTuple(color)

        input_poly_data.GetPointData().SetScalars(colors)  # command to color the mesh based on colors array

        # Create a mapper and actor
        mapper = vtk.vtkPolyDataMapper()  # mapper dedicated to the mesh
        mapper.SetInputData(input_poly_data)  # input Data from the colored mesh
        actor = vtk.vtkActor()  # define a actor for the mesh in the scene to be rendered
        actor.SetMapper(mapper)  # sets the actors mapper as the mesh mapper

        line_actors_list = []
        index = 0

        if isinstance(self.graphs_metrics, list):
            actor_line = self.line_actor("TxtPaths/DijkstraPoints" + self.graphs_metrics[3] + ".txt", path_colors[0])
            line_actors_list.append(actor_line)

        elif isinstance(self.graphs_metrics, tuple):
            for graphs in self.graphs_metrics:
                actor_line = self.line_actor("TxtPaths/DijkstraPoints" + graphs[3] + ".txt", path_colors[index])
                line_actors_list.append(actor_line)
                index += 1

        # Create a renderer, render window, and interactor
        renderer = vtk.vtkRenderer()
        render_window = vtk.vtkRenderWindow()
        render_window.AddRenderer(renderer)
        render_window_interactor = vtk.vtkRenderWindowInteractor()
        render_window_interactor.SetRenderWindow(render_window)

        style = vtkInteractorStyleTrackballCamera()
        style.SetCurrentRenderer(renderer)
        render_window_interactor.SetInteractorStyle(style)

        axes_actor = vtk.vtkAxesActor()
        axes_actor.AxisLabelsOn()
        axes_actor.SetShaftTypeToCylinder()
        axes_actor.SetCylinderRadius(0.05)

        # Add the actor to the scene
        renderer.AddActor(actor)
        renderer.AddActor(axes_actor)

        for actors in line_actors_list:  # adds the actors (lines) from source point adjacency
            renderer.AddActor(actors)

        renderer.SetBackground(.1, .2, .3)

        # Render and interact
        render_window.Render()
        render_window_interactor.Start()

    def run(self):          # run the graphs to all metrics and returns the dijkstra path list

        processes_list = []
        plot_colors = []
        dijkstra_paths = []

        os.system("mkdir TxtPaths")

        if isinstance(self.graphs_metrics, list):   # if user is processing only one metric
            try:            # plot the map if a color is passed
                plot_colors.append(self.graphs_metrics[4])
                self.run_graph_process(self.graphs_metrics[0], self.graphs_metrics[1], self.graphs_metrics[2],
                                       self.graphs_metrics[3])
                dijkstra = self.file_to_list("TxtPaths/dijkstraPath" + self.graphs_metrics[3] + ".txt")
                dijkstra_paths.append(dijkstra)
                self.print_path_properties(dijkstra, self.graphs_metrics[3])

            except IndexError:      # no color assigned so the class does not plot the map with paths
                self.run_graph_process(self.graphs_metrics[0], self.graphs_metrics[1], self.graphs_metrics[2],
                                       self.graphs_metrics[3])
                dijkstra = self.file_to_list("TxtPaths/dijkstraPath" + self.graphs_metrics[3] + ".txt")
                dijkstra_paths.append(dijkstra)
                self.print_path_properties(dijkstra, self.graphs_metrics[3])

        if isinstance(self.graphs_metrics, tuple):  # if user is processing multiple metrics
            for graph_metric in self.graphs_metrics:
                graph_proc = Process(target=self.run_graph_process, args=(graph_metric[0], graph_metric[1],
                                                                          graph_metric[2], graph_metric[3]))
                try:
                    plot_colors.append(graph_metric[4])
                    processes_list.append(graph_proc)

                except IndexError:
                    processes_list.append(graph_proc)

            for process in processes_list:
                process.start()

            for process in processes_list:
                process.join()

            for graph in self.graphs_metrics:
                dijkstra = self.file_to_list("TxtPaths/dijkstraPath" + graph[3] + ".txt")
                dijkstra_paths.append(dijkstra)
                self.print_path_properties(dijkstra, graph[3])

        if len(plot_colors) > 0:     # if the list of colors is empty, it does not plot the map
            self.plot_path(plot_colors)

        return dijkstra_paths

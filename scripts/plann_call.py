import sys
import ros
import pymesh
import math
import numpy as np
from MeshPathFinder import *
from scipy.spatial import cKDTree

from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D
import random


def find_face_centroid(centroids, xyz_):
    source_face = -1
    indx = 0

    min_dist = 999999999999

    print("xyz_:", xyz_, "centroid_size:", len(centroids))

    for face in centroids:
        #, rel_tol=1e-2
        d = math.sqrt((face[0] - xyz_[0]) ** 2 + (face[1] - xyz_[1])**2 + (face[2] - xyz_[2])**2)
        if d < min_dist:
            min_dist = d
            source_face = indx
            print("min_dist:", min_dist)

        if  np.isclose(face[0], xyz_[0], rtol=1.e-2, atol=1.e-2) and \
            np.isclose(face[1], xyz_[1], rtol=1.e-2, atol=1.e-2) and \
            np.isclose(face[2], xyz_[2], rtol=1.e-2, atol=1.e-2):
            source_face = indx
            print("find direct tolerance:", min_dist)
            break
        indx += 1

    print("Returned face idx:", source_face)
    return source_face


def main(argv):
    mesh_path = argv[0]
    xyz_source = [float(argv[1]), float(argv[2]), float(argv[3])]
    xyz_target = [float(argv[4]), float(argv[5]), float(argv[6])]

    mesh_load = pymesh.load_mesh(mesh_path)
    mesh_load.add_attribute("face_centroid")
    centroids = mesh_load.get_face_attribute("face_centroid")

    # fig = pyplot.figure()
    # ax = Axes3D(fig)

    # x, y, z = zip(*centroids)
    # ax.scatter(x, y, z)
    # pyplot.show()

    print(centroids)
    vertices = mesh_load.vertices
    ver_face = mesh_load.faces

    source_face = find_face_centroid(centroids, xyz_source)
    target_face = find_face_centroid(centroids, xyz_target)

    if source_face == -1 or target_face == -1:
        print("ERROR! Face not found in mesh.")
        sys.exit()

    graph_metrics = ([1, source_face, target_face, "Shortest"],
                     [2, source_face, target_face, "Transverse"],
                     [3, source_face, target_face, "Energy"],
                     [4, source_face, target_face, "Combined"])

    planner = MeshPathFinder(mesh_path, graph_metrics)
    planner.run()


if __name__ == "__main__":
    main(sys.argv[1:])

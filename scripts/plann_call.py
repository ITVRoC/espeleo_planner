import sys
import pymesh
import math
from MeshPathFinder import *


def find_face_vertice(vertices, xy_):
    face = -1
    indx = 0
    for vertice in vertices:
        if math.isclose(vertice[0], xy_[0], rel_tol=1e-1) and math.isclose(vertice[1], xy_[1], rel_tol=1e-1):
            face = indx
            break
        indx += 1
    return face


def find_face_centroid(centroids, xy_):
    source_face = -1
    indx = 0
    for face in centroids:
        if math.isclose(face[0], xy_[0], rel_tol=1e-2) and math.isclose(face[1], xy_[1], rel_tol=1e-2):
            source_face = indx
            break
        indx += 1
    return source_face


def main(argv):
    mesh_path = argv[0]
    xy_source = [float(argv[1]), float(argv[2])]
    xy_target = [float(argv[3]), float(argv[4])]

    mesh_load = pymesh.load_mesh(mesh_path)
    mesh_load.add_attribute("face_centroid")
    centroids = mesh_load.get_face_attribute("face_centroid")
    vertices = mesh_load.vertices
    ver_face = mesh_load.faces

    source_face = find_face_centroid(centroids, xy_source)
    target_face = find_face_centroid(centroids, xy_target)

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

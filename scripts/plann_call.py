import getopt
import sys
from MeshPathFinder import *
import pymesh
import math


def main(argv):
    # print(argv)
    mesh_path = argv[0]
    xy_source = [float(argv[1]), float(argv[2])]
    xy_target = [float(argv[3]), float(argv[4])]
    print(mesh_path, xy_source, xy_target)
    mesh_load = pymesh.load_mesh(mesh_path)
    mesh_load.add_attribute("face_centroid")
    centroids = mesh_load.get_face_attribute("face_centroid")
    print(centroids)
    print(mesh_load.num_faces)
    source_face = 0
    indx = 0
    for face in centroids:
        if math.isclose(face[0], xy_source[0], rel_tol=1e-3) and math.isclose(face[1], xy_source[1], rel_tol=1e-3):
            source_face = indx
            break
        indx += 1
    target_face = 0
    indx = 0
    for face in centroids:
        if math.isclose(face[0], xy_target[0], rel_tol=1e-3) and math.isclose(face[1], xy_target[1], rel_tol=1e-3):
            target_face = indx
            break
        indx += 1

    graph_metrics = ([1, source_face, target_face, "Shortest"],
                     [2, source_face, target_face, "Transverse"],
                     [3, source_face, target_face, "Energy"],
                     [4, source_face, target_face, "Combined"])

    planner = MeshPathFinder(mesh_path, graph_metrics)
    planner.run()


if __name__ == "__main__":
    main(sys.argv[1:])

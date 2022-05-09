import open3d as o3d
import networkx as nx
import numpy as np
#from mayavi import mlab
import itertools


def plot_graph_3d(G, centroids, title=None, source_id=None, target_id=None, border_3d_points=None,
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

    g_centroids = [tuple(centroids[v]) for v in sorted(G.nodes())]
    centroid_gcon_dict = {v: int(i) for i, v in enumerate(g_centroids)}

    xyz = np.array(g_centroids)
    scalars = xyz[:, 2]
    pts = mlab.points3d(xyz[:, 0], xyz[:, 1], xyz[:, 2],
                        scalars,
                        scale_factor=0.01,
                        scale_mode='none',
                        colormap='Blues',
                        resolution=20)

    edge_list = []
    for e in G.edges():
        e1 = tuple(centroids[e[0]])
        e2 = tuple(centroids[e[1]])
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
        src_3d = centroids[source_id]
        mlab.text(src_3d[0], src_3d[1], "source", z=src_3d[2], width=0.2)
        mlab.points3d([src_3d[0]], [src_3d[1]], [src_3d[2]],
                        scale_factor=0.25,
                        scale_mode='none',
                        color=(0, 1.0, 0.1),
                        resolution=20)

    if target_id:
        tgt_3d = centroids[target_id]
        mlab.text(tgt_3d[0], tgt_3d[1], "target", z=tgt_3d[2], width=0.2)
        mlab.points3d([tgt_3d[0]], [tgt_3d[1]], [tgt_3d[2]],
                        scale_factor=0.25,
                        scale_mode='none',
                        color=(0, 0.1, 1.0),
                        resolution=20)

    if reachable_frontiers_ids and len(reachable_frontiers_ids) > 0:
        frontiers_3dp = [tuple(centroids[v]) for v in reachable_frontiers_ids]
        xyz = np.array(frontiers_3dp)
        mlab.points3d(xyz[:, 0], xyz[:, 1], xyz[:, 2],
                        scale_factor=0.15,
                        scale_mode='none',
                        color=(1.0, 0.1, 1.0),
                        resolution=20)

    if frontier_centroids_ids and len(frontier_centroids_ids) > 0:
        centroids_3dp = [tuple(centroids[v]) for v in frontier_centroids_ids]
        xyz = np.array(centroids_3dp)
        mlab.points3d(xyz[:, 0], xyz[:, 1], xyz[:, 2],
                        scale_factor=0.35,
                        scale_mode='none',
                        color=(1.0, 0.1, 1.0),
                        resolution=20)

    if frontier_visit_ids and len(frontier_visit_ids) > 0:
        centroids_3dp = [tuple(centroids[v]) for v in frontier_visit_ids]
        xyz = np.array(centroids_3dp)
        mlab.points3d(xyz[:, 0], xyz[:, 1], xyz[:, 2],
                        scale_factor=0.35,
                        scale_mode='none',
                        color=(1.0, 0.1, 1.0),
                        resolution=20)

    mlab.show()

def enumerate2(xs, start=0, step=1):
    for x in xs:
        yield (start, x)
        start += step

mesh_filepath = "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/recon_surface/tmp_reconstructions/04-04-2022_17:48:49_mesh.stl"

mesh = o3d.io.read_triangle_mesh(mesh_filepath)
mesh.compute_adjacency_list()
adj_list = mesh.adjacency_list
verts = np.asarray(mesh.vertices)
faces = np.asarray(mesh.triangles)

point_to_id = {}
id_to_point = {}

idx_counter = 0
for v in verts:
    v_tuple = tuple(v)
    if v_tuple not in point_to_id:
        point_to_id[v_tuple] = idx_counter
        id_to_point[idx_counter] = v_tuple
        idx_counter += 1

print("point_to_id:", len(point_to_id.keys()))
print("verts:", len(verts))

print(len(adj_list))

faces = []
#for idx, vert in enumerate2(adj_list, start=0, step=3):
for idx in range(0, len(adj_list), 3):
    vert = adj_list[idx]
    vert.add(idx)
    vert_list = list(vert)
    
    print("idx:", idx, vert_list)

    local_face = []
    for v_id in vert_list:
        point_3d = verts[v_id]
        v_tuple = tuple(point_3d)
        local_face.append(point_to_id[v_tuple])

    faces.append(local_face)

print("faces:", len(faces))


#centroids = list(verts)

centroids = []
for k in sorted(id_to_point.keys()):
    centroids.append(id_to_point[k])

#create graph
edges = []
for face in faces:
    edges.extend(list(itertools.combinations(face, 2)))
#G = nx.from_edgelist(edges)

G = nx.Graph()
for e in edges:
    G.add_edge(e[0], e[1], weight=1)

# compute connected components and print results
components = list(nx.algorithms.components.connected_components(G))

for component in components:
    print(component)
    G_conn = G.subgraph(component).copy()
    plot_graph_3d(G_conn, centroids)
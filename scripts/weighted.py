# -*- coding: utf-8 -*-
#    Copyright (C) 2004-2018 by
#    Aric Hagberg <hagberg@lanl.gov>
#    Dan Schult <dschult@colgate.edu>
#    Pieter Swart <swart@lanl.gov>
#    All rights reserved.
#    BSD license.
#
# Authors:  Aric Hagberg <hagberg@lanl.gov>
#           Loïc Séguin-C. <loicseguin@gmail.com>
#           Dan Schult <dschult@colgate.edu>
#           Niels van Adrichem <n.l.m.vanadrichem@tudelft.nl>


# File must be replaced in: /usr/local/lib/python2.7/dist-packages/networkx/algorithms/shortest_path

"""
Shortest path algorithms for weighed graphs.
"""

from collections import deque
from heapq import heappush, heappop
from itertools import count
import networkx as nx
from networkx.utils import generate_unique_node
import math
import numpy as np
# custom imports:
import pybullet as p
from BibMathEns import *


__all__ = ['dijkstra_path',
           'dijkstra_path_length',
           'bidirectional_dijkstra',
           'single_source_dijkstra',
           'single_source_dijkstra_path',
           'single_source_dijkstra_path_length',
           'multi_source_dijkstra',
           'multi_source_dijkstra_path',
           'multi_source_dijkstra_path_length',
           'all_pairs_dijkstra',
           'all_pairs_dijkstra_path',
           'all_pairs_dijkstra_path_length',
           'dijkstra_predecessor_and_distance',
           'bellman_ford_path',
           'bellman_ford_path_length',
           'single_source_bellman_ford',
           'single_source_bellman_ford_path',
           'single_source_bellman_ford_path_length',
           'all_pairs_bellman_ford_path',
           'all_pairs_bellman_ford_path_length',
           'bellman_ford_predecessor_and_distance',
           'negative_edge_cycle',
           'goldberg_radzik',
           'johnson']


def _weight_function(G, weight):
    """Returns a function that returns the weight of an edge.

    The returned function is specifically suitable for input to
    functions :func:`_dijkstra` and :func:`_bellman_ford_relaxation`.

    Parameters
    ----------
    G : NetworkX graph.

    weight : string or function
        If it is callable, `weight` itself is returned. If it is a string,
        it is assumed to be the name of the edge attribute that represents
        the weight of an edge. In that case, a function is returned that
        gets the edge weight according to the specified edge attribute.

    Returns
    -------
    function
        This function returns a callable that accepts exactly three inputs:
        a node, an node adjacent to the first one, and the edge attribute
        dictionary for the eedge joining those nodes. That function returns
        a number representing the weight of an edge.

    If `G` is a multigraph, and `weight` is not callable, the
    minimum edge weight over all parallel edges is returned. If any edge
    does not have an attribute with key `weight`, it is assumed to
    have weight one.

    """
    if callable(weight):
        return weight
    # If the weight keyword argument is not callable, we assume it is a
    # string representing the edge attribute containing the weight of
    # the edge.
    if G.is_multigraph():
        return lambda u, v, d: min(attr.get(weight, 1) for attr in d.values())
    return lambda u, v, data: data.get(weight, 1)


def dijkstra_path(G, source, target, centroids, normals, minMaxList, short_w, energ_w, transv_w, metric,
                  robot_id=None, map_id=None, bullet_client=None, weight='weight'):

    """Returns the shortest weighted path from source to target in G.

    Uses Dijkstra's Method to compute the shortest weighted path
    between two nodes in a graph.

    Parameters
    ----------
    G : NetworkX graph

    source : node
       Starting node

    target : node
       Ending node

    weight : string or function
       If this is a string, then edge weights will be accessed via the
       edge attribute with this key (that is, the weight of the edge
       joining `u` to `v` will be ``G.edges[u, v][weight]``). If no
       such edge attribute exists, the weight of the edge is assumed to
       be one.

       If this is a function, the weight of an edge is the value
       returned by the function. The function must accept exactly three
       positional arguments: the two endpoints of an edge and the
       dictionary of edge attributes for that edge. The function must
       return a number.

    Returns
    -------
    path : list
       List of nodes in a shortest path.

    Raises
    ------
    NodeNotFound
        If `source` is not in `G`.

    NetworkXNoPath
       If no path exists between source and target.

    Examples
    --------
    >>> G=nx.path_graph(5)
    >>> print(nx.dijkstra_path(G,0,4))
    [0, 1, 2, 3, 4]

    Notes
    -----
    Edge weight attributes must be numerical.
    Distances are calculated as sums of weighted edges traversed.

    The weight function can be used to hide edges by returning None.
    So ``weight = lambda u, v, d: 1 if d['color']=="red" else None``
    will find the shortest red path.

    The weight function can be used to include node weights.

    >>> def func(u, v, d):
    ...     node_u_wt = G.nodes[u].get('node_weight', 1)
    ...     node_v_wt = G.nodes[v].get('node_weight', 1)
    ...     edge_wt = d.get('weight', 1)
    ...     return node_u_wt/2 + node_v_wt/2 + edge_wt

    In this example we take the average of start and end node
    weights of an edge and add it to the weight of the edge.

    See Also
    --------
    bidirectional_dijkstra(), bellman_ford_path()
    """
    (length, path) = single_source_dijkstra(G, source,  centroids, normals, minMaxList, short_w, energ_w, transv_w,
                                            metric, robot_id=robot_id, map_id=map_id, bullet_client=bullet_client,
                                            target=target, weight=weight)
    return path


def dijkstra_path_length(G, source, target, weight='weight'):
    """Returns the shortest weighted path length in G from source to target.

    Uses Dijkstra's Method to compute the shortest weighted path length
    between two nodes in a graph.

    Parameters
    ----------
    G : NetworkX graph

    source : node label
       starting node for path

    target : node label
       ending node for path

    weight : string or function
       If this is a string, then edge weights will be accessed via the
       edge attribute with this key (that is, the weight of the edge
       joining `u` to `v` will be ``G.edges[u, v][weight]``). If no
       such edge attribute exists, the weight of the edge is assumed to
       be one.

       If this is a function, the weight of an edge is the value
       returned by the function. The function must accept exactly three
       positional arguments: the two endpoints of an edge and the
       dictionary of edge attributes for that edge. The function must
       return a number.

    Returns
    -------
    length : number
        Shortest path length.

    Raises
    ------
    NodeNotFound
        If `source` is not in `G`.

    NetworkXNoPath
        If no path exists between source and target.

    Examples
    --------
    >>> G=nx.path_graph(5)
    >>> print(nx.dijkstra_path_length(G,0,4))
    4

    Notes
    -----
    Edge weight attributes must be numerical.
    Distances are calculated as sums of weighted edges traversed.

    The weight function can be used to hide edges by returning None.
    So ``weight = lambda u, v, d: 1 if d['color']=="red" else None``
    will find the shortest red path.

    See Also
    --------
    bidirectional_dijkstra(), bellman_ford_path_length()

    """
    if source == target:
        return 0
    weight = _weight_function(G, weight)
    length = _dijkstra(G, source, weight, target=target)
    try:
        return length[target]
    except KeyError:
        raise nx.NetworkXNoPath(
            "Node %s not reachable from %s" % (target, source))


def single_source_dijkstra_path(G, source, cutoff=None, weight='weight'):
    """Find shortest weighted paths in G from a source node.

    Compute shortest path between source and all other reachable
    nodes for a weighted graph.

    Parameters
    ----------
    G : NetworkX graph

    source : node
       Starting node for path.

    cutoff : integer or float, optional
       Depth to stop the search. Only return paths with length <= cutoff.

    weight : string or function
       If this is a string, then edge weights will be accessed via the
       edge attribute with this key (that is, the weight of the edge
       joining `u` to `v` will be ``G.edges[u, v][weight]``). If no
       such edge attribute exists, the weight of the edge is assumed to
       be one.

       If this is a function, the weight of an edge is the value
       returned by the function. The function must accept exactly three
       positional arguments: the two endpoints of an edge and the
       dictionary of edge attributes for that edge. The function must
       return a number.

    Returns
    -------
    paths : dictionary
       Dictionary of shortest path lengths keyed by target.

    Raises
    ------
    NodeNotFound
        If `source` is not in `G`.

    Examples
    --------
    >>> G=nx.path_graph(5)
    >>> path=nx.single_source_dijkstra_path(G,0)
    >>> path[4]
    [0, 1, 2, 3, 4]

    Notes
    -----
    Edge weight attributes must be numerical.
    Distances are calculated as sums of weighted edges traversed.

    The weight function can be used to hide edges by returning None.
    So ``weight = lambda u, v, d: 1 if d['color']=="red" else None``
    will find the shortest red path.

    See Also
    --------
    single_source_dijkstra(), single_source_bellman_ford()

    """
    return multi_source_dijkstra_path(G, {source}, cutoff=cutoff,
                                      weight=weight)


def single_source_dijkstra_path_length(G, source, cutoff=None,
                                       weight='weight'):
    """Find shortest weighted path lengths in G from a source node.

    Compute the shortest path length between source and all other
    reachable nodes for a weighted graph.

    Parameters
    ----------
    G : NetworkX graph

    source : node label
       Starting node for path

    cutoff : integer or float, optional
       Depth to stop the search. Only return paths with length <= cutoff.

    weight : string or function
       If this is a string, then edge weights will be accessed via the
       edge attribute with this key (that is, the weight of the edge
       joining `u` to `v` will be ``G.edges[u, v][weight]``). If no
       such edge attribute exists, the weight of the edge is assumed to
       be one.

       If this is a function, the weight of an edge is the value
       returned by the function. The function must accept exactly three
       positional arguments: the two endpoints of an edge and the
       dictionary of edge attributes for that edge. The function must
       return a number.

    Returns
    -------
    length : dict
        Dict keyed by node to shortest path length from source.

    Raises
    ------
    NodeNotFound
        If `source` is not in `G`.

    Examples
    --------
    >>> G = nx.path_graph(5)
    >>> length = nx.single_source_dijkstra_path_length(G, 0)
    >>> length[4]
    4
    >>> for node in [0, 1, 2, 3, 4]:
    ...     print('{}: {}'.format(node, length[node]))
    0: 0
    1: 1
    2: 2
    3: 3
    4: 4

    Notes
    -----
    Edge weight attributes must be numerical.
    Distances are calculated as sums of weighted edges traversed.

    The weight function can be used to hide edges by returning None.
    So ``weight = lambda u, v, d: 1 if d['color']=="red" else None``
    will find the shortest red path.

    See Also
    --------
    single_source_dijkstra(), single_source_bellman_ford_path_length()

    """
    return multi_source_dijkstra_path_length(G, {source}, cutoff=cutoff,
                                             weight=weight)


def single_source_dijkstra(G, source, centroids, normals, minMaxList, short_w, energ_w, transv_w, metric,
                           robot_id=None, map_id=None, bullet_client=None, target=None, cutoff=None,  weight='weight'):

    """Find shortest weighted paths and lengths from a source node.

    Compute the shortest path length between source and all other
    reachable nodes for a weighted graph.

    Uses Dijkstra's algorithm to compute shortest paths and lengths
    between a source and all other reachable nodes in a weighted graph.

    Parameters
    ----------
    G : NetworkX graph

    source : node label
       Starting node for path

    target : node label, optional
       Ending node for path

    cutoff : integer or float, optional
       Depth to stop the search. Only return paths with length <= cutoff.

    weight : string or function
       If this is a string, then edge weights will be accessed via the
       edge attribute with this key (that is, the weight of the edge
       joining `u` to `v` will be ``G.edges[u, v][weight]``). If no
       such edge attribute exists, the weight of the edge is assumed to
       be one.

       If this is a function, the weight of an edge is the value
       returned by the function. The function must accept exactly three
       positional arguments: the two endpoints of an edge and the
       dictionary of edge attributes for that edge. The function must
       return a number.

    Returns
    -------
    distance, path : pair of dictionaries, or numeric and list.
       If target is None, paths and lengths to all nodes are computed.
       The return value is a tuple of two dictionaries keyed by target nodes.
       The first dictionary stores distance to each target node.
       The second stores the path to each target node.
       If target is not None, returns a tuple (distance, path), where
       distance is the distance from source to target and path is a list
       representing the path from source to target.

    Raises
    ------
    NodeNotFound
        If `source` is not in `G`.

    Examples
    --------
    >>> G = nx.path_graph(5)
    >>> length, path = nx.single_source_dijkstra(G, 0)
    >>> print(length[4])
    4
    >>> for node in [0, 1, 2, 3, 4]:
    ...     print('{}: {}'.format(node, length[node]))
    0: 0
    1: 1
    2: 2
    3: 3
    4: 4
    >>> path[4]
    [0, 1, 2, 3, 4]
    >>> length, path = nx.single_source_dijkstra(G, 0, 1)
    >>> length
    1
    >>> path
    [0, 1]

    Notes
    -----
    Edge weight attributes must be numerical.
    Distances are calculated as sums of weighted edges traversed.

    The weight function can be used to hide edges by returning None.
    So ``weight = lambda u, v, d: 1 if d['color']=="red" else None``
    will find the shortest red path.

    Based on the Python cookbook recipe (119466) at
    http://aspn.activestate.com/ASPN/Cookbook/Python/Recipe/119466

    This algorithm is not guaranteed to work if edge weights
    are negative or are floating point numbers
    (overflows and roundoff errors can cause problems).

    See Also
    --------
    single_source_dijkstra_path()
    single_source_dijkstra_path_length()
    single_source_bellman_ford()
    """
    return multi_source_dijkstra(G, {source}, centroids, normals, minMaxList, short_w, energ_w, transv_w,
                                 metric, robot_id=robot_id, map_id=map_id, bullet_client=bullet_client,
                                 cutoff=cutoff, target=target, weight=weight)


def multi_source_dijkstra_path(G, sources, cutoff=None, weight='weight'):
    """Find shortest weighted paths in G from a given set of source
    nodes.

    Compute shortest path between any of the source nodes and all other
    reachable nodes for a weighted graph.

    Parameters
    ----------
    G : NetworkX graph

    sources : non-empty set of nodes
        Starting nodes for paths. If this is just a set containing a
        single node, then all paths computed by this function will start
        from that node. If there are two or more nodes in the set, the
        computed paths may begin from any one of the start nodes.

    cutoff : integer or float, optional
       Depth to stop the search. Only return paths with length <= cutoff.

    weight : string or function
       If this is a string, then edge weights will be accessed via the
       edge attribute with this key (that is, the weight of the edge
       joining `u` to `v` will be ``G.edges[u, v][weight]``). If no
       such edge attribute exists, the weight of the edge is assumed to
       be one.

       If this is a function, the weight of an edge is the value
       returned by the function. The function must accept exactly three
       positional arguments: the two endpoints of an edge and the
       dictionary of edge attributes for that edge. The function must
       return a number.

    Returns
    -------
    paths : dictionary
       Dictionary of shortest paths keyed by target.

    Examples
    --------
    >>> G = nx.path_graph(5)
    >>> path = nx.multi_source_dijkstra_path(G, {0, 4})
    >>> path[1]
    [0, 1]
    >>> path[3]
    [4, 3]

    Notes
    -----
    Edge weight attributes must be numerical.
    Distances are calculated as sums of weighted edges traversed.

    The weight function can be used to hide edges by returning None.
    So ``weight = lambda u, v, d: 1 if d['color']=="red" else None``
    will find the shortest red path.

    Raises
    ------
    ValueError
        If `sources` is empty.
    NodeNotFound
        If any of `sources` is not in `G`.

    See Also
    --------
    multi_source_dijkstra(), multi_source_bellman_ford()

    """
    length, path = multi_source_dijkstra(G, sources, cutoff=cutoff,
                                         weight=weight)
    return path


def multi_source_dijkstra_path_length(G, sources, cutoff=None,
                                      weight='weight'):
    """Find shortest weighted path lengths in G from a given set of
    source nodes.

    Compute the shortest path length between any of the source nodes and
    all other reachable nodes for a weighted graph.

    Parameters
    ----------
    G : NetworkX graph

    sources : non-empty set of nodes
        Starting nodes for paths. If this is just a set containing a
        single node, then all paths computed by this function will start
        from that node. If there are two or more nodes in the set, the
        computed paths may begin from any one of the start nodes.

    cutoff : integer or float, optional
       Depth to stop the search. Only return paths with length <= cutoff.

    weight : string or function
       If this is a string, then edge weights will be accessed via the
       edge attribute with this key (that is, the weight of the edge
       joining `u` to `v` will be ``G.edges[u, v][weight]``). If no
       such edge attribute exists, the weight of the edge is assumed to
       be one.

       If this is a function, the weight of an edge is the value
       returned by the function. The function must accept exactly three
       positional arguments: the two endpoints of an edge and the
       dictionary of edge attributes for that edge. The function must
       return a number.

    Returns
    -------
    length : dict
        Dict keyed by node to shortest path length to nearest source.

    Examples
    --------
    >>> G = nx.path_graph(5)
    >>> length = nx.multi_source_dijkstra_path_length(G, {0, 4})
    >>> for node in [0, 1, 2, 3, 4]:
    ...     print('{}: {}'.format(node, length[node]))
    0: 0
    1: 1
    2: 2
    3: 1
    4: 0

    Notes
    -----
    Edge weight attributes must be numerical.
    Distances are calculated as sums of weighted edges traversed.

    The weight function can be used to hide edges by returning None.
    So ``weight = lambda u, v, d: 1 if d['color']=="red" else None``
    will find the shortest red path.

    Raises
    ------
    ValueError
        If `sources` is empty.
    NodeNotFound
        If any of `sources` is not in `G`.

    See Also
    --------
    multi_source_dijkstra()

    """
    if not sources:
        raise ValueError('sources must not be empty')
    weight = _weight_function(G, weight)
    return _dijkstra_multisource(G, sources, weight, cutoff=cutoff)


def multi_source_dijkstra(G, sources, centroids, normals, minMaxList, short_w, energ_w, transv_w,
                          metric, robot_id=None, map_id=None, bullet_client=None, target=None,
                          cutoff=None, weight='weight'):

    """Find shortest weighted paths and lengths from a given set of
    source nodes.

    Uses Dijkstra's algorithm to compute the shortest paths and lengths
    between one of the source nodes and the given `target`, or all other
    reachable nodes if not specified, for a weighted graph.

    Parameters
    ----------
    G : NetworkX graph

    sources : non-empty set of nodes
        Starting nodes for paths. If this is just a set containing a
        single node, then all paths computed by this function will start
        from that node. If there are two or more nodes in the set, the
        computed paths may begin from any one of the start nodes.

    target : node label, optional
       Ending node for path

    cutoff : integer or float, optional
       Depth to stop the search. Only return paths with length <= cutoff.

    weight : string or function
       If this is a string, then edge weights will be accessed via the
       edge attribute with this key (that is, the weight of the edge
       joining `u` to `v` will be ``G.edges[u, v][weight]``). If no
       such edge attribute exists, the weight of the edge is assumed to
       be one.

       If this is a function, the weight of an edge is the value
       returned by the function. The function must accept exactly three
       positional arguments: the two endpoints of an edge and the
       dictionary of edge attributes for that edge. The function must
       return a number.

    Returns
    -------
    distance, path : pair of dictionaries, or numeric and list
       If target is None, returns a tuple of two dictionaries keyed by node.
       The first dictionary stores distance from one of the source nodes.
       The second stores the path from one of the sources to that node.
       If target is not None, returns a tuple of (distance, path) where
       distance is the distance from source to target and path is a list
       representing the path from source to target.

    Examples
    --------
    >>> G = nx.path_graph(5)
    >>> length, path = nx.multi_source_dijkstra(G, {0, 4})
    >>> for node in [0, 1, 2, 3, 4]:
    ...     print('{}: {}'.format(node, length[node]))
    0: 0
    1: 1
    2: 2
    3: 1
    4: 0
    >>> path[1]
    [0, 1]
    >>> path[3]
    [4, 3]

    >>> length, path = nx.multi_source_dijkstra(G, {0, 4}, 1)
    >>> length
    1
    >>> path
    [0, 1]

    Notes
    -----
    Edge weight attributes must be numerical.
    Distances are calculated as sums of weighted edges traversed.

    The weight function can be used to hide edges by returning None.
    So ``weight = lambda u, v, d: 1 if d['color']=="red" else None``
    will find the shortest red path.

    Based on the Python cookbook recipe (119466) at
    http://aspn.activestate.com/ASPN/Cookbook/Python/Recipe/119466

    This algorithm is not guaranteed to work if edge weights
    are negative or are floating point numbers
    (overflows and roundoff errors can cause problems).

    Raises
    ------
    ValueError
        If `sources` is empty.
    NodeNotFound
        If any of `sources` is not in `G`.

    See Also
    --------
    multi_source_dijkstra_path()
    multi_source_dijkstra_path_length()

    """
    # this method runs only once
    if not sources:
        raise ValueError('sources must not be empty')
    if target in sources:
        return (0, [target])
    weight = _weight_function(G, weight)
    paths = {source: [source] for source in sources}  # dictionary of paths
    dist = _dijkstra_multisource(G, sources, weight, centroids, normals, minMaxList, short_w, energ_w, transv_w,
                                 metric, robot_id=robot_id, map_id=map_id, bullet_client=bullet_client,
                                 paths=paths, cutoff=cutoff, target=target)

    if target is None:
        return (dist, paths)
    try:
        return (dist[target], paths[target])
    except KeyError:
        raise nx.NetworkXNoPath("No path to {}.".format(target))


def _dijkstra(G, source, weight, pred=None, paths=None, cutoff=None,
              target=None):
    """Uses Dijkstra's algorithm to find shortest weighted paths from a
    single source.

    This is a convenience function for :func:`_dijkstra_multisource`
    with all the arguments the same, except the keyword argument
    `sources` set to ``[source]``.

    """
    return _dijkstra_multisource(G, [source], weight, pred=pred, paths=paths,
                                 cutoff=cutoff, target=target)


def euclidianDistanceWeight(nodeSource, nodeTarget, centroids):    # calculates the euclidian distance between two nodes, based on its centroids[x,y,z]
    sourcePoints = centroids[nodeSource]
    targetPoints = centroids[nodeTarget]
    deltaX = math.pow((np.float128(sourcePoints[0]) - np.float128(targetPoints[0])), 2)
    deltaY = math.pow((np.float128(sourcePoints[1]) - np.float128(targetPoints[1])), 2)
    deltaZ = math.pow((np.float128(sourcePoints[2]) - np.float128(targetPoints[2])), 2)
    sum = deltaX + deltaY + deltaZ
    sum = np.float128(sum)
    lenght = math.sqrt(sum)
    lenght = np.float128(lenght)
    return lenght


def angle2Vectors(vector1, vector2):       # returns the angle in degrees
    scalar = np.dot(vector1, vector2)  # scalar product between the face normal and Z axe
    norms = np.linalg.norm(vector1) * np.linalg.norm(vector2)  # norm between face normal and Z Axe
    if norms == 0:
        cosValue = 0
    else:
        cosValue = scalar / norms

    if cosValue > 1:        # treating floating point problems
        cosValue = 1

    elif cosValue < -1:     # treating floating point problems
        cosValue = -1

    radians = math.acos(cosValue)  # arccos of scalar/norms
    degrees = math.degrees(radians)
    return degrees


def terrainWeight(nodeTarget, normals):
    sourceNormalP = normals[nodeTarget]            # face source normal
    zAxe = [0, 0, -1]                                 # vector representing the Z axe
    scalar = np.dot(sourceNormalP, zAxe)           # scalar product between the face normal and Z axe
    norms = np.linalg.norm(sourceNormalP) * np.linalg.norm(zAxe)  # norm between face normal and Z Axe
    cosAngle = scalar/norms
    radians = math.acos(cosAngle)                  # arccos of scalar/norms
    angleDegrees = math.degrees(radians)
    angleDegrees = math.fabs(angleDegrees)
    return angleDegrees


def energyWeight(nodeSource, nodeTarget, predecessor, centroids, final):
    pointsSource = centroids[nodeSource]            # points of the center of the source face (node)
    pointsTarget = centroids[nodeTarget]            # points of the center of the target face (node)

    vector2Faces = [pointsTarget[0] - pointsSource[0], pointsTarget[1] - pointsSource[1],
                    pointsTarget[2] - pointsSource[2]]    # vector between the the face and target points

    if predecessor == -1:
        pointsPredecessor = centroids[final]
        vectorP2C = [pointsPredecessor[0] - pointsSource[0],
                     pointsPredecessor[1] - pointsSource[1]]  # vector between the predecessor and current node

    else:
        pointsPredecessor = centroids[predecessor]  # points [x, y, z] of the predecessor node
        vectorP2C = [pointsSource[0] - pointsPredecessor[0], pointsSource[1] - pointsPredecessor[1]]


    vectorC2T = [pointsTarget[0] - pointsSource[0],
                 pointsTarget[1] - pointsSource[1]]  # vector between the current and target node

    rot = angle2Vectors(vectorP2C, vectorC2T)  # returns the minor angle between the two vectors, representing
    # the rotation between the two faces

    vectorXY = [0, 0, -1]                                  # normal of the XY plan = Z Axe
    angle = angle2Vectors(vector2Faces, vectorXY)         # gets the angle between the two vectors
    angle = 90 - angle
    dist = euclidianDistanceWeight(nodeSource, nodeTarget, centroids)   # distance between the faces (nodes)
    # rot = angleRotation(pointsSource, pointsTarget)     # rotation between the two faces
    if angle < 0:
        energy = (((37735.9 * rot)/360) + ((-475.07*angle) + 1089.3)) * dist
    else:
        energy = (((37735.9 * rot)/360) + ((564.97*angle) + 1364.9)) * dist

    return energy


def normalizeLength(length, maxDist, minDist):
    normaLengthData = ((length - minDist) / (maxDist - minDist)) * ((10 - 1) + 1)
    return normaLengthData


def normalizeRotation(rotation, maxRot, minRot):
    normaRotData = ((rotation - minRot)/(maxRot - minRot)) * ((10 - 1) + 1)
    return normaRotData


def normalizeTransversal(transversal, maxTra, minTra):
    if transversal == 0:
        return 1
    normalTraData = ((transversal - minTra) / (maxTra - minTra)) * ((10 - 1) + 1)
    return normalTraData


def normalizeEnergy(energy, maxEnergy, minEnergy):
    normalEnergData = ((energy - minEnergy) / (maxEnergy - minEnergy)) * ((10 - 1) + 1)
    return normalEnergData


def calcRotation(nodeSource, nodeTarget, predecessor, centroids, final):    # returns the rotation in angles
    pointsCurrent = centroids[nodeSource]  # points [x, y, z] of the current node
    pointsTarget = centroids[nodeTarget]  # points [x, y, z] of the target node

    if predecessor == -1:
        pointsPredecessor = centroids[final]
        vectorP2C = [pointsPredecessor[0] - pointsCurrent[0],
                     pointsPredecessor[1] - pointsCurrent[1]]  # vector between the predecessor and current node

    else:
        pointsPredecessor = centroids[predecessor]  # points [x, y, z] of the predecessor node
        vectorP2C = [pointsCurrent[0] - pointsPredecessor[0], pointsCurrent[1] - pointsPredecessor[1]]


    vectorC2T = [pointsTarget[0] - pointsCurrent[0],
                 pointsTarget[1] - pointsCurrent[1]]  # vector between the current and target node

    rotation = angle2Vectors(vectorP2C, vectorC2T)
    return rotation


def distance(p0, p1):
    return math.sqrt(math.pow((p0[0] - p1[0]), 2) + math.pow((p0[1] - p1[1]), 2))


def rollover_angles(bullet_client, map_id, robot_id, x, y, z, robot_yaw):

    robot_start_pos = [x, y, z]
    robot_start_orientation = bullet_client.getQuaternionFromEuler([-math.pi / 2, 0, robot_yaw])
    # robot_start_orientation = bullet_client.getQuaternionFromEuler([0, 0, robot_yaw])
    bullet_client.resetBasePositionAndOrientation(robot_id, robot_start_pos, robot_start_orientation)

    bullet_client.stepSimulation()
    contact_points = bullet_client.getContactPoints(robot_id)
    len_cpoints = len(contact_points)

    while len_cpoints > 0:
        robot_start_pos[2] = robot_start_pos[2] + 0.1
        bullet_client.resetBasePositionAndOrientation(robot_id, robot_start_pos, robot_start_orientation)
        bullet_client.stepSimulation()
        contact_points = bullet_client.getContactPoints(robot_id)
        len_cpoints = len(contact_points)

    for i in range(500):

        bullet_client.stepSimulation()
        robot_pos, robot_orient = bullet_client.getBasePositionAndOrientation(robot_id)
        robot_orient = bullet_client.getEulerFromQuaternion(robot_orient)
        contact_points = bullet_client.getContactPoints(robot_id)
        len_cpoints = len(contact_points)
        dist = distance(robot_start_pos, robot_pos)

        if dist > 0.15:     # stops the simulation if the distance between starting position and current position
            return -1, -1           # is bigger than 0.15

        for j in range(len_cpoints):        # stops the simulation if the robots chassi hits the ground
            if contact_points[j][3] == -1:
                return -1, -1

        if (robot_orient[0] > (robot_start_orientation[0] + math.pi / 2)) \
                or (robot_orient[0] < (robot_start_orientation[0] - math.pi / 2)) \
                or (robot_orient[1] > (robot_start_orientation[1] + math.pi / 2)) \
                or (robot_orient[1] < (robot_start_orientation[1] - math.pi / 2)):  # or (robot_pos[2] < 0):
            return -1, -1           # stops the simulation if robot rollover

    pcon_r0 = p.getClosestPoints(robot_id, map_id, 0.2, 0)  # Gets the link 0 closest points of terrain
    pcon_r1 = p.getClosestPoints(robot_id, map_id, 0.2, 1)  # Gets the link 1 closest points of terrain
    pcon_r2 = p.getClosestPoints(robot_id, map_id, 0.2, 2)  # Gets the link 2 closest points of terrain
    pcon_r3 = p.getClosestPoints(robot_id, map_id, 0.2, 3)  # Gets the link 3 closest points of terrain
    pcon_r4 = p.getClosestPoints(robot_id, map_id, 0.2, 4)  # Gets the link 4 closest points of terrain
    pcon_r5 = p.getClosestPoints(robot_id, map_id, 0.2, 5)  # Gets the link 5 closest points of terrain

    p_r0 = []
    p_r1 = []
    p_r2 = []
    p_r3 = []
    p_r4 = []
    p_r5 = []

    if len(pcon_r0) > 0:
        p_r0 = pcon_r0[0][6]
    if len(pcon_r1) > 0:
        p_r1 = pcon_r1[0][6]
    if len(pcon_r2) > 0:
        p_r2 = pcon_r2[0][6]
    if len(pcon_r3) > 0:
        p_r3 = pcon_r3[0][6]
    if len(pcon_r4) > 0:
        p_r4 = pcon_r4[0][6]
    if len(pcon_r5) > 0:
        p_r5 = pcon_r5[0][6]

    xyz_pcontato_ord = [p_r0, p_r1, p_r2, p_r3, p_r4, p_r5]     # appending the lists
    xyz_pcontato_ord = list(filter(None, xyz_pcontato_ord))     # pops the empty elements on list

    robot_pos, robot_orient = bullet_client.getBasePositionAndOrientation(robot_id)

    return xyz_pcontato_ord, robot_pos


def edgeWeight(nodeSource, nodeTarget, centroids, normals, minMaxList, predecessor, final, short_w, energ_w, transv_w,
               metric, robot_id=None, map_id=None, bullet_client=None):

    if metric == 1:
        length = euclidianDistanceWeight(nodeSource, nodeTarget, centroids)  # calculate euclidian distance between two nodes,
        return length

    if metric == 2:
        length = euclidianDistanceWeight(nodeSource, nodeTarget, centroids)
        transversal = terrainWeight(nodeTarget, normals)
        return transversal + length

    if metric == 3:
        if predecessor == -1:
            predecessor = final
        energySpent = energyWeight(nodeSource, nodeTarget, predecessor, centroids, final)
        return energySpent

    if metric == 4:
        length = euclidianDistanceWeight(nodeSource, nodeTarget,
                                         centroids)  # calculate euclidian distance between two nodes
        lenghtCost = normalizeLength(length, minMaxList[0], minMaxList[1])
        transversal = terrainWeight(nodeTarget, normals)    # calculate angle between Z Axe and face normal
        transversalCost = normalizeTransversal(transversal, minMaxList[4], minMaxList[5])
        energySpent = energyWeight(nodeSource, nodeTarget, predecessor, centroids, final)
        energyCost = normalizeEnergy(energySpent, minMaxList[6], minMaxList[7])

        rotation = calcRotation(nodeSource, nodeTarget, predecessor, centroids, final)
        weightC = ((short_w * lenghtCost)+rotation) + (transv_w * transversalCost) + (energ_w * energyCost)
        return weightC

    if metric == 5:
        rotation = calcRotation(nodeSource, nodeTarget, predecessor, centroids, final)
        return rotation

    if metric == 6:
        calc_obj = bib_mathEnsaio.instance()     # object to call the rollover calculus
        vet_seq_lig = [0, 1, 2, 3, 4, 5]       # list to index the robots wheels
        x, y, z = centroids[nodeSource]        # coordinate to put robot into
        yaw = calcRotation(nodeSource, nodeTarget, predecessor, centroids, final)  # robot yaw(deg) for reference in map
        yaw = np.radians(yaw)
        contact_points, points_mc = rollover_angles(bullet_client, map_id, robot_id, x, y, z, yaw)
        if contact_points == -1 or not contact_points:    # if robot rollover or too dangerous to pass
            return np.inf           # returns infinite so the path will never go through there
        # uses pybullet to get the robot contact points with terrain
        min_angle = calc_obj.calculaEnergiaEstabilidade(vet_seq_lig, contact_points, points_mc, 17.81, -9.8)
        # gets the array with min rollover angle and points, energy data, center of mass, intersection points
        min_angle = min_angle[1]    # gets the minimal angle from previous array
        min_angle = abs(min_angle)
        angle_cost = 90 - min_angle    # actual cost from node to target
        return angle_cost


def _dijkstra_multisource(G, sources, weight, centroids, normals, minMaxList, short_w, energ_w, transv_w,
                          metric, robot_id=None, map_id=None, bullet_client=None, pred=None, paths=None,
                          cutoff=None, target=None):

    """Uses Dijkstra's algorithm to find shortest weighted paths

    Parameters
    ----------
    G : NetworkX graph

    sources : non-empty iterable of nodes
        Starting nodes for paths. If this is just an iterable containing
        a single node, then all paths computed by this function will
        start from that node. If there are two or more nodes in this
        iterable, the computed paths may begin from any one of the start
        nodes.

    weight: function
        Function with (u, v, data) input that returns that edges weight

    pred: dict of lists, optional(default=None)
        dict to store a list of predecessors keyed by that node
        If None, predecessors are not stored.

    paths: dict, optional (default=None)
        dict to store the path list from source to each node, keyed by node.
        If None, paths are not stored.

    target : node label, optional
        Ending node for path. Search is halted when target is found.

    cutoff : integer or float, optional
        Depth to stop the search. Only return paths with length <= cutoff.

    Returns
    -------
    distance : dictionary
        A mapping from node to shortest distance to that node from one
        of the source nodes.

    Raises
    ------
    NodeNotFound
        If any of `sources` is not in `G`.

    Notes
    -----
    The optional predecessor and path dictionaries can be accessed by
    the caller through the original pred and paths objects passed
    as arguments. No need to explicitly return pred or paths.

    """
    G_succ = G._succ if G.is_directed() else G._adj
    push = heappush
    pop = heappop
    dist = {}  # dictionary of final distances
    seen = {}
    # fringe is heapq with 3-tuples (distance,c,node)
    # use the count c to avoid comparing nodes (may not be able to)
    c = count()
    fringe = []
    initialNode = 0
    for source in sources:
        if source not in G:
            raise nx.NodeNotFound("Source {} not in G".format(source))
        seen[source] = 0
        push(fringe, (0, next(c), source))
        initialNode = source

    while fringe:
        (d, _, v) = pop(fringe)   # this v is the next node to path

        if v == initialNode:
            predecessor = -1            # if node == to source node of search
        else:
            predecessor = paths[v][-2]      # predecessor == last adjacent node (adj to v)in path

        if v in dist:
            continue  # already searched this node.
        dist[v] = d
        if v == target:
            break
        for u, e in G_succ[v].items():
            # cost = weight(v, u, e)     # original cost function from v to u

            if (robot_id and map_id and bullet_client) is not None:
                cost = edgeWeight(v, u, centroids, normals, minMaxList, predecessor, target, short_w, energ_w, transv_w,
                                  metric, robot_id, map_id, bullet_client)  # modded weight func
            else:
                cost = edgeWeight(v, u, centroids, normals, minMaxList, predecessor, target, short_w, energ_w,
                                  transv_w, metric)  # modded weight func

            if cost is None:
                continue
            vu_dist = dist[v] + cost
            if cutoff is not None:
                if vu_dist > cutoff:
                    continue
            if u in dist:
                if vu_dist < dist[u]:
                    raise ValueError('Contradictory paths found:',
                                     'negative weights?')
            elif u not in seen or vu_dist < seen[u]:
                seen[u] = vu_dist           # seen[u] = a weight
                push(fringe, (vu_dist, next(c), u))
                if paths is not None:
                    paths[u] = paths[v] + [u]
                if pred is not None:
                    pred[u] = [v]
            elif vu_dist == seen[u]:
                if pred is not None:
                    pred[u].append(v)

    # The optional predecessor and path dictionaries can be accessed
    # by the caller via the pred and paths objects passed as arguments.
    return dist


def dijkstra_predecessor_and_distance(G, source, cutoff=None, weight='weight'):
    """Compute weighted shortest path length and predecessors.

    Uses Dijkstra's Method to obtain the shortest weighted paths
    and return dictionaries of predecessors for each node and
    distance for each node from the `source`.

    Parameters
    ----------
    G : NetworkX graph

    source : node label
       Starting node for path

    cutoff : integer or float, optional
       Depth to stop the search. Only return paths with length <= cutoff.

    weight : string or function
       If this is a string, then edge weights will be accessed via the
       edge attribute with this key (that is, the weight of the edge
       joining `u` to `v` will be ``G.edges[u, v][weight]``). If no
       such edge attribute exists, the weight of the edge is assumed to
       be one.

       If this is a function, the weight of an edge is the value
       returned by the function. The function must accept exactly three
       positional arguments: the two endpoints of an edge and the
       dictionary of edge attributes for that edge. The function must
       return a number.

    Returns
    -------
    pred, distance : dictionaries
       Returns two dictionaries representing a list of predecessors
       of a node and the distance to each node.
       Warning: If target is specified, the dicts are incomplete as they
       only contain information for the nodes along a path to target.

    Raises
    ------
    NodeNotFound
        If `source` is not in `G`.

    Notes
    -----
    Edge weight attributes must be numerical.
    Distances are calculated as sums of weighted edges traversed.

    The list of predecessors contains more than one element only when
    there are more than one shortest paths to the key node.

    Examples
    --------
    >>> import networkx as nx
    >>> G = nx.path_graph(5, create_using = nx.DiGraph())
    >>> pred, dist = nx.dijkstra_predecessor_and_distance(G, 0)
    >>> sorted(pred.items())
    [(0, []), (1, [0]), (2, [1]), (3, [2]), (4, [3])]
    >>> sorted(dist.items())
    [(0, 0), (1, 1), (2, 2), (3, 3), (4, 4)]

    >>> pred, dist = nx.dijkstra_predecessor_and_distance(G, 0, 1)
    >>> sorted(pred.items())
    [(0, []), (1, [0])]
    >>> sorted(dist.items())
    [(0, 0), (1, 1)]
    """

    weight = _weight_function(G, weight)
    pred = {source: []}  # dictionary of predecessors
    return (pred, _dijkstra(G, source, weight, pred=pred, cutoff=cutoff))


def all_pairs_dijkstra(G, cutoff=None, weight='weight'):
    """Find shortest weighted paths and lengths between all nodes.

    Parameters
    ----------
    G : NetworkX graph

    cutoff : integer or float, optional
       Depth to stop the search. Only return paths with length <= cutoff.

    weight : string or function
       If this is a string, then edge weights will be accessed via the
       edge attribute with this key (that is, the weight of the edge
       joining `u` to `v` will be ``G.edge[u][v][weight]``). If no
       such edge attribute exists, the weight of the edge is assumed to
       be one.

       If this is a function, the weight of an edge is the value
       returned by the function. The function must accept exactly three
       positional arguments: the two endpoints of an edge and the
       dictionary of edge attributes for that edge. The function must
       return a number.

    Yields
    ------
    (node, (distance, path)) : (node obj, (dict, dict))
        Each source node has two associated dicts. The first holds distance
        keyed by target and the second holds paths keyed by target.
        (See single_source_dijkstra for the source/target node terminology.)
        If desired you can apply `dict()` to this function to create a dict
        keyed by source node to the two dicts.

    Examples
    --------
    >>> G = nx.path_graph(5)
    >>> len_path = dict(nx.all_pairs_dijkstra(G))
    >>> print(len_path[3][0][1])
    2
    >>> for node in [0, 1, 2, 3, 4]:
    ...     print('3 - {}: {}'.format(node, len_path[3][0][node]))
    3 - 0: 3
    3 - 1: 2
    3 - 2: 1
    3 - 3: 0
    3 - 4: 1
    >>> len_path[3][1][1]
    [3, 2, 1]
    >>> for n, (dist, path) in nx.all_pairs_dijkstra(G):
    ...     print(path[1])
    [0, 1]
    [1]
    [2, 1]
    [3, 2, 1]
    [4, 3, 2, 1]

    Notes
    -----
    Edge weight attributes must be numerical.
    Distances are calculated as sums of weighted edges traversed.

    The yielded dicts only have keys for reachable nodes.
    """
    for n in G:
        dist, path = single_source_dijkstra(G, n, cutoff=cutoff, weight=weight)
        yield (n, (dist, path))


def all_pairs_dijkstra_path_length(G, cutoff=None, weight='weight'):
    """Compute shortest path lengths between all nodes in a weighted graph.

    Parameters
    ----------
    G : NetworkX graph

    cutoff : integer or float, optional
       Depth to stop the search. Only return paths with length <= cutoff.

    weight : string or function
       If this is a string, then edge weights will be accessed via the
       edge attribute with this key (that is, the weight of the edge
       joining `u` to `v` will be ``G.edges[u, v][weight]``). If no
       such edge attribute exists, the weight of the edge is assumed to
       be one.

       If this is a function, the weight of an edge is the value
       returned by the function. The function must accept exactly three
       positional arguments: the two endpoints of an edge and the
       dictionary of edge attributes for that edge. The function must
       return a number.

    Returns
    -------
    distance : iterator
        (source, dictionary) iterator with dictionary keyed by target and
        shortest path length as the key value.

    Examples
    --------
    >>> G = nx.path_graph(5)
    >>> length = dict(nx.all_pairs_dijkstra_path_length(G))
    >>> for node in [0, 1, 2, 3, 4]:
    ...     print('1 - {}: {}'.format(node, length[1][node]))
    1 - 0: 1
    1 - 1: 0
    1 - 2: 1
    1 - 3: 2
    1 - 4: 3
    >>> length[3][2]
    1
    >>> length[2][2]
    0

    Notes
    -----
    Edge weight attributes must be numerical.
    Distances are calculated as sums of weighted edges traversed.

    The dictionary returned only has keys for reachable node pairs.
    """
    length = single_source_dijkstra_path_length
    for n in G:
        yield (n, length(G, n, cutoff=cutoff, weight=weight))


def all_pairs_dijkstra_path(G, cutoff=None, weight='weight'):
    """Compute shortest paths between all nodes in a weighted graph.

    Parameters
    ----------
    G : NetworkX graph

    cutoff : integer or float, optional
       Depth to stop the search. Only return paths with length <= cutoff.

    weight : string or function
       If this is a string, then edge weights will be accessed via the
       edge attribute with this key (that is, the weight of the edge
       joining `u` to `v` will be ``G.edges[u, v][weight]``). If no
       such edge attribute exists, the weight of the edge is assumed to
       be one.

       If this is a function, the weight of an edge is the value
       returned by the function. The function must accept exactly three
       positional arguments: the two endpoints of an edge and the
       dictionary of edge attributes for that edge. The function must
       return a number.

    Returns
    -------
    distance : dictionary
       Dictionary, keyed by source and target, of shortest paths.

    Examples
    --------
    >>> G = nx.path_graph(5)
    >>> path = dict(nx.all_pairs_dijkstra_path(G))
    >>> print(path[0][4])
    [0, 1, 2, 3, 4]

    Notes
    -----
    Edge weight attributes must be numerical.
    Distances are calculated as sums of weighted edges traversed.

    See Also
    --------
    floyd_warshall(), all_pairs_bellman_ford_path()

    """
    path = single_source_dijkstra_path
    # TODO This can be trivially parallelized.
    for n in G:
        yield (n, path(G, n, cutoff=cutoff, weight=weight))


def bellman_ford_predecessor_and_distance(G, source, target=None,
                                          weight='weight'):
    """Compute shortest path lengths and predecessors on shortest paths
    in weighted graphs.

    The algorithm has a running time of $O(mn)$ where $n$ is the number of
    nodes and $m$ is the number of edges.  It is slower than Dijkstra but
    can handle negative edge weights.

    Parameters
    ----------
    G : NetworkX graph
       The algorithm works for all types of graphs, including directed
       graphs and multigraphs.

    source: node label
       Starting node for path

    weight : string or function
       If this is a string, then edge weights will be accessed via the
       edge attribute with this key (that is, the weight of the edge
       joining `u` to `v` will be ``G.edges[u, v][weight]``). If no
       such edge attribute exists, the weight of the edge is assumed to
       be one.

       If this is a function, the weight of an edge is the value
       returned by the function. The function must accept exactly three
       positional arguments: the two endpoints of an edge and the
       dictionary of edge attributes for that edge. The function must
       return a number.

    Returns
    -------
    pred, dist : dictionaries
       Returns two dictionaries keyed by node to predecessor in the
       path and to the distance from the source respectively.

    Raises
    ------
    NodeNotFound
        If `source` is not in `G`.

    NetworkXUnbounded
       If the (di)graph contains a negative cost (di)cycle, the
       algorithm raises an exception to indicate the presence of the
       negative cost (di)cycle.  Note: any negative weight edge in an
       undirected graph is a negative cost cycle.

    Examples
    --------
    >>> import networkx as nx
    >>> G = nx.path_graph(5, create_using = nx.DiGraph())
    >>> pred, dist = nx.bellman_ford_predecessor_and_distance(G, 0)
    >>> sorted(pred.items())
    [(0, []), (1, [0]), (2, [1]), (3, [2]), (4, [3])]
    >>> sorted(dist.items())
    [(0, 0), (1, 1), (2, 2), (3, 3), (4, 4)]

    >>> pred, dist = nx.bellman_ford_predecessor_and_distance(G, 0, 1)
    >>> sorted(pred.items())
    [(0, []), (1, [0]), (2, [1]), (3, [2]), (4, [3])]
    >>> sorted(dist.items())
    [(0, 0), (1, 1), (2, 2), (3, 3), (4, 4)]

    >>> from nose.tools import assert_raises
    >>> G = nx.cycle_graph(5, create_using = nx.DiGraph())
    >>> G[1][2]['weight'] = -7
    >>> assert_raises(nx.NetworkXUnbounded, \
                      nx.bellman_ford_predecessor_and_distance, G, 0)

    Notes
    -----
    Edge weight attributes must be numerical.
    Distances are calculated as sums of weighted edges traversed.

    The dictionaries returned only have keys for nodes reachable from
    the source.

    In the case where the (di)graph is not connected, if a component
    not containing the source contains a negative cost (di)cycle, it
    will not be detected.

    In NetworkX v2.1 and prior, the source node had predecessor `[None]`.
    In NetworkX v2.2 this changed to the source node having predecessor `[]`
    """
    if source not in G:
        raise nx.NodeNotFound("Node %s is not found in the graph" % source)
    weight = _weight_function(G, weight)
    if any(weight(u, v, d) < 0 for u, v, d in nx.selfloop_edges(G, data=True)):
        raise nx.NetworkXUnbounded("Negative cost cycle detected.")

    dist = {source: 0}
    pred = {source: []}

    if len(G) == 1:
        return pred, dist

    weight = _weight_function(G, weight)

    dist = _bellman_ford(G, [source], weight, pred=pred, dist=dist,
                         target=target)
    return (pred, dist)


def _bellman_ford(G, source, weight, pred=None, paths=None, dist=None,
                  target=None):
    """Relaxation loop for Bellman–Ford algorithm.

    This is an implementation of the SPFA variant.
    See https://en.wikipedia.org/wiki/Shortest_Path_Faster_Algorithm

    Parameters
    ----------
    G : NetworkX graph

    source: list
        List of source nodes. The shortest path from any of the source
        nodes will be found if multiple sources are provided.

    weight : function
        The weight of an edge is the value returned by the function. The
        function must accept exactly three positional arguments: the two
        endpoints of an edge and the dictionary of edge attributes for
        that edge. The function must return a number.

    pred: dict of lists, optional (default=None)
        dict to store a list of predecessors keyed by that node
        If None, predecessors are not stored

    paths: dict, optional (default=None)
        dict to store the path list from source to each node, keyed by node
        If None, paths are not stored

    dist: dict, optional (default=None)
        dict to store distance from source to the keyed node
        If None, returned dist dict contents default to 0 for every node in the
        source list

    target: node label, optional
        Ending node for path. Path lengths to other destinations may (and
        probably will) be incorrect.

    Returns
    -------
    Returns a dict keyed by node to the distance from the source.
    Dicts for paths and pred are in the mutated input dicts by those names.

    Raises
    ------
    NodeNotFound
        If any of `source` is not in `G`.

    NetworkXUnbounded
       If the (di)graph contains a negative cost (di)cycle, the
       algorithm raises an exception to indicate the presence of the
       negative cost (di)cycle.  Note: any negative weight edge in an
       undirected graph is a negative cost cycle
    """
    for s in source:
        if s not in G:
            raise nx.NodeNotFound("Source {} not in G".format(s))

    if pred is None:
        pred = {v: [] for v in source}

    if dist is None:
        dist = {v: 0 for v in source}

    G_succ = G.succ if G.is_directed() else G.adj
    inf = float('inf')
    n = len(G)

    count = {}
    q = deque(source)
    in_q = set(source)
    while q:
        u = q.popleft()
        in_q.remove(u)

        # Skip relaxations if any of the predecessors of u is in the queue.
        if all(pred_u not in in_q for pred_u in pred[u]):
            dist_u = dist[u]
            for v, e in G_succ[u].items():
                dist_v = dist_u + weight(v, u, e)

                if dist_v < dist.get(v, inf):
                    if v not in in_q:
                        q.append(v)
                        in_q.add(v)
                        count_v = count.get(v, 0) + 1
                        if count_v == n:
                            raise nx.NetworkXUnbounded(
                                "Negative cost cycle detected.")
                        count[v] = count_v
                    dist[v] = dist_v
                    pred[v] = [u]

                elif dist.get(v) is not None and dist_v == dist.get(v):
                    pred[v].append(u)

    if paths is not None:
        dsts = [target] if target is not None else pred
        for dst in dsts:

            path = [dst]
            cur = dst

            while pred[cur]:
                cur = pred[cur][0]
                path.append(cur)

            path.reverse()
            paths[dst] = path

    return dist


def bellman_ford_path(G, source, target, weight='weight'):
    """Returns the shortest path from source to target in a weighted graph G.

    Parameters
    ----------
    G : NetworkX graph

    source : node
       Starting node

    target : node
       Ending node

    weight: string, optional (default='weight')
       Edge data key corresponding to the edge weight

    Returns
    -------
    path : list
       List of nodes in a shortest path.

    Raises
    ------
    NodeNotFound
        If `source` is not in `G`.

    NetworkXNoPath
       If no path exists between source and target.

    Examples
    --------
    >>> G=nx.path_graph(5)
    >>> print(nx.bellman_ford_path(G, 0, 4))
    [0, 1, 2, 3, 4]

    Notes
    -----
    Edge weight attributes must be numerical.
    Distances are calculated as sums of weighted edges traversed.

    See Also
    --------
    dijkstra_path(), bellman_ford_path_length()
    """
    length, path = single_source_bellman_ford(G, source,
                                              target=target, weight=weight)
    return path


def bellman_ford_path_length(G, source, target, weight='weight'):
    """Returns the shortest path length from source to target
    in a weighted graph.

    Parameters
    ----------
    G : NetworkX graph

    source : node label
       starting node for path

    target : node label
       ending node for path

    weight: string, optional (default='weight')
       Edge data key corresponding to the edge weight

    Returns
    -------
    length : number
        Shortest path length.

    Raises
    ------
    NodeNotFound
        If `source` is not in `G`.

    NetworkXNoPath
        If no path exists between source and target.

    Examples
    --------
    >>> G=nx.path_graph(5)
    >>> print(nx.bellman_ford_path_length(G,0,4))
    4

    Notes
    -----
    Edge weight attributes must be numerical.
    Distances are calculated as sums of weighted edges traversed.

    See Also
    --------
    dijkstra_path_length(), bellman_ford_path()
    """
    if source == target:
        return 0

    weight = _weight_function(G, weight)

    length = _bellman_ford(G, [source], weight, target=target)

    try:
        return length[target]
    except KeyError:
        raise nx.NetworkXNoPath(
            "node %s not reachable from %s" % (source, target))


def single_source_bellman_ford_path(G, source, weight='weight'):
    """Compute shortest path between source and all other reachable
    nodes for a weighted graph.

    Parameters
    ----------
    G : NetworkX graph

    source : node
       Starting node for path.

    weight: string, optional (default='weight')
       Edge data key corresponding to the edge weight

    Returns
    -------
    paths : dictionary
       Dictionary of shortest path lengths keyed by target.

    Raises
    ------
    NodeNotFound
        If `source` is not in `G`.

    Examples
    --------
    >>> G=nx.path_graph(5)
    >>> path=nx.single_source_bellman_ford_path(G,0)
    >>> path[4]
    [0, 1, 2, 3, 4]

    Notes
    -----
    Edge weight attributes must be numerical.
    Distances are calculated as sums of weighted edges traversed.

    See Also
    --------
    single_source_dijkstra(), single_source_bellman_ford()

    """
    (length, path) = single_source_bellman_ford(
        G, source, weight=weight)
    return path


def single_source_bellman_ford_path_length(G, source, weight='weight'):
    """Compute the shortest path length between source and all other
    reachable nodes for a weighted graph.

    Parameters
    ----------
    G : NetworkX graph

    source : node label
       Starting node for path

    weight: string, optional (default='weight')
       Edge data key corresponding to the edge weight.

    Returns
    -------
    length : iterator
        (target, shortest path length) iterator

    Raises
    ------
    NodeNotFound
        If `source` is not in `G`.

    Examples
    --------
    >>> G = nx.path_graph(5)
    >>> length = dict(nx.single_source_bellman_ford_path_length(G, 0))
    >>> length[4]
    4
    >>> for node in [0, 1, 2, 3, 4]:
    ...     print('{}: {}'.format(node, length[node]))
    0: 0
    1: 1
    2: 2
    3: 3
    4: 4

    Notes
    -----
    Edge weight attributes must be numerical.
    Distances are calculated as sums of weighted edges traversed.

    See Also
    --------
    single_source_dijkstra(), single_source_bellman_ford()

    """
    weight = _weight_function(G, weight)
    return _bellman_ford(G, [source], weight)


def single_source_bellman_ford(G, source, target=None, weight='weight'):
    """Compute shortest paths and lengths in a weighted graph G.

    Uses Bellman-Ford algorithm for shortest paths.

    Parameters
    ----------
    G : NetworkX graph

    source : node label
       Starting node for path

    target : node label, optional
       Ending node for path

    Returns
    -------
    distance, path : pair of dictionaries, or numeric and list
       If target is None, returns a tuple of two dictionaries keyed by node.
       The first dictionary stores distance from one of the source nodes.
       The second stores the path from one of the sources to that node.
       If target is not None, returns a tuple of (distance, path) where
       distance is the distance from source to target and path is a list
       representing the path from source to target.

    Raises
    ------
    NodeNotFound
        If `source` is not in `G`.

    Examples
    --------
    >>> G = nx.path_graph(5)
    >>> length, path = nx.single_source_bellman_ford(G, 0)
    >>> print(length[4])
    4
    >>> for node in [0, 1, 2, 3, 4]:
    ...     print('{}: {}'.format(node, length[node]))
    0: 0
    1: 1
    2: 2
    3: 3
    4: 4
    >>> path[4]
    [0, 1, 2, 3, 4]
    >>> length, path = nx.single_source_bellman_ford(G, 0, 1)
    >>> length
    1
    >>> path
    [0, 1]

    Notes
    -----
    Edge weight attributes must be numerical.
    Distances are calculated as sums of weighted edges traversed.

    See Also
    --------
    single_source_dijkstra()
    single_source_bellman_ford_path()
    single_source_bellman_ford_path_length()
    """
    if source == target:
        return (0, [source])

    weight = _weight_function(G, weight)

    paths = {source: [source]}  # dictionary of paths
    dist = _bellman_ford(G, [source], weight, paths=paths, target=target)
    if target is None:
        return (dist, paths)
    try:
        return (dist[target], paths[target])
    except KeyError:
        msg = "Node %s not reachable from %s" % (source, target)
        raise nx.NetworkXNoPath(msg)


def all_pairs_bellman_ford_path_length(G, weight='weight'):
    """ Compute shortest path lengths between all nodes in a weighted graph.

    Parameters
    ----------
    G : NetworkX graph

    weight: string, optional (default='weight')
       Edge data key corresponding to the edge weight

    Returns
    -------
    distance : iterator
        (source, dictionary) iterator with dictionary keyed by target and
        shortest path length as the key value.

    Examples
    --------
    >>> G = nx.path_graph(5)
    >>> length = dict(nx.all_pairs_bellman_ford_path_length(G))
    >>> for node in [0, 1, 2, 3, 4]:
    ...     print('1 - {}: {}'.format(node, length[1][node]))
    1 - 0: 1
    1 - 1: 0
    1 - 2: 1
    1 - 3: 2
    1 - 4: 3
    >>> length[3][2]
    1
    >>> length[2][2]
    0

    Notes
    -----
    Edge weight attributes must be numerical.
    Distances are calculated as sums of weighted edges traversed.

    The dictionary returned only has keys for reachable node pairs.
    """
    length = single_source_bellman_ford_path_length
    for n in G:
        yield (n, dict(length(G, n, weight=weight)))


def all_pairs_bellman_ford_path(G, weight='weight'):
    """ Compute shortest paths between all nodes in a weighted graph.

    Parameters
    ----------
    G : NetworkX graph

    weight: string, optional (default='weight')
       Edge data key corresponding to the edge weight

    Returns
    -------
    distance : dictionary
       Dictionary, keyed by source and target, of shortest paths.

    Examples
    --------
    >>> G = nx.path_graph(5)
    >>> path = dict(nx.all_pairs_bellman_ford_path(G))
    >>> print(path[0][4])
    [0, 1, 2, 3, 4]

    Notes
    -----
    Edge weight attributes must be numerical.
    Distances are calculated as sums of weighted edges traversed.

    See Also
    --------
    floyd_warshall(), all_pairs_dijkstra_path()

    """
    path = single_source_bellman_ford_path
    # TODO This can be trivially parallelized.
    for n in G:
        yield (n, path(G, n, weight=weight))


def goldberg_radzik(G, source, weight='weight'):
    """Compute shortest path lengths and predecessors on shortest paths
    in weighted graphs.

    The algorithm has a running time of $O(mn)$ where $n$ is the number of
    nodes and $m$ is the number of edges.  It is slower than Dijkstra but
    can handle negative edge weights.

    Parameters
    ----------
    G : NetworkX graph
       The algorithm works for all types of graphs, including directed
       graphs and multigraphs.

    source: node label
       Starting node for path

    weight : string or function
       If this is a string, then edge weights will be accessed via the
       edge attribute with this key (that is, the weight of the edge
       joining `u` to `v` will be ``G.edges[u, v][weight]``). If no
       such edge attribute exists, the weight of the edge is assumed to
       be one.

       If this is a function, the weight of an edge is the value
       returned by the function. The function must accept exactly three
       positional arguments: the two endpoints of an edge and the
       dictionary of edge attributes for that edge. The function must
       return a number.

    Returns
    -------
    pred, dist : dictionaries
       Returns two dictionaries keyed by node to predecessor in the
       path and to the distance from the source respectively.

    Raises
    ------
    NodeNotFound
        If `source` is not in `G`.

    NetworkXUnbounded
       If the (di)graph contains a negative cost (di)cycle, the
       algorithm raises an exception to indicate the presence of the
       negative cost (di)cycle.  Note: any negative weight edge in an
       undirected graph is a negative cost cycle.

    Examples
    --------
    >>> import networkx as nx
    >>> G = nx.path_graph(5, create_using = nx.DiGraph())
    >>> pred, dist = nx.goldberg_radzik(G, 0)
    >>> sorted(pred.items())
    [(0, None), (1, 0), (2, 1), (3, 2), (4, 3)]
    >>> sorted(dist.items())
    [(0, 0), (1, 1), (2, 2), (3, 3), (4, 4)]

    >>> from nose.tools import assert_raises
    >>> G = nx.cycle_graph(5, create_using = nx.DiGraph())
    >>> G[1][2]['weight'] = -7
    >>> assert_raises(nx.NetworkXUnbounded, nx.goldberg_radzik, G, 0)

    Notes
    -----
    Edge weight attributes must be numerical.
    Distances are calculated as sums of weighted edges traversed.

    The dictionaries returned only have keys for nodes reachable from
    the source.

    In the case where the (di)graph is not connected, if a component
    not containing the source contains a negative cost (di)cycle, it
    will not be detected.

    """
    if source not in G:
        raise nx.NodeNotFound("Node %s is not found in the graph" % source)
    weight = _weight_function(G, weight)
    if any(weight(u, v, d) < 0 for u, v, d in nx.selfloop_edges(G, data=True)):
        raise nx.NetworkXUnbounded("Negative cost cycle detected.")

    if len(G) == 1:
        return {source: None}, {source: 0}

    if G.is_directed():
        G_succ = G.succ
    else:
        G_succ = G.adj

    inf = float('inf')
    d = {u: inf for u in G}
    d[source] = 0
    pred = {source: None}

    def topo_sort(relabeled):
        """Topologically sort nodes relabeled in the previous round and detect
        negative cycles.
        """
        # List of nodes to scan in this round. Denoted by A in Goldberg and
        # Radzik's paper.
        to_scan = []
        # In the DFS in the loop below, neg_count records for each node the
        # number of edges of negative reduced costs on the path from a DFS root
        # to the node in the DFS forest. The reduced cost of an edge (u, v) is
        # defined as d[u] + weight[u][v] - d[v].
        #
        # neg_count also doubles as the DFS visit marker array.
        neg_count = {}
        for u in relabeled:
            # Skip visited nodes.
            if u in neg_count:
                continue
            d_u = d[u]
            # Skip nodes without out-edges of negative reduced costs.
            if all(d_u + weight(u, v, e) >= d[v]
                   for v, e in G_succ[u].items()):
                continue
            # Nonrecursive DFS that inserts nodes reachable from u via edges of
            # nonpositive reduced costs into to_scan in (reverse) topological
            # order.
            stack = [(u, iter(G_succ[u].items()))]
            in_stack = set([u])
            neg_count[u] = 0
            while stack:
                u, it = stack[-1]
                try:
                    v, e = next(it)
                except StopIteration:
                    to_scan.append(u)
                    stack.pop()
                    in_stack.remove(u)
                    continue
                t = d[u] + weight(u, v, e)
                d_v = d[v]
                if t <= d_v:
                    is_neg = t < d_v
                    d[v] = t
                    pred[v] = u
                    if v not in neg_count:
                        neg_count[v] = neg_count[u] + int(is_neg)
                        stack.append((v, iter(G_succ[v].items())))
                        in_stack.add(v)
                    elif (v in in_stack and
                          neg_count[u] + int(is_neg) > neg_count[v]):
                        # (u, v) is a back edge, and the cycle formed by the
                        # path v to u and (u, v) contains at least one edge of
                        # negative reduced cost. The cycle must be of negative
                        # cost.
                        raise nx.NetworkXUnbounded(
                            'Negative cost cycle detected.')
        to_scan.reverse()
        return to_scan

    def relax(to_scan):
        """Relax out-edges of relabeled nodes.
        """
        relabeled = set()
        # Scan nodes in to_scan in topological order and relax incident
        # out-edges. Add the relabled nodes to labeled.
        for u in to_scan:
            d_u = d[u]
            for v, e in G_succ[u].items():
                w_e = weight(u, v, e)
                if d_u + w_e < d[v]:
                    d[v] = d_u + w_e
                    pred[v] = u
                    relabeled.add(v)
        return relabeled

    # Set of nodes relabled in the last round of scan operations. Denoted by B
    # in Goldberg and Radzik's paper.
    relabeled = set([source])

    while relabeled:
        to_scan = topo_sort(relabeled)
        relabeled = relax(to_scan)

    d = {u: d[u] for u in pred}
    return pred, d


def negative_edge_cycle(G, weight='weight'):
    """Return True if there exists a negative edge cycle anywhere in G.

    Parameters
    ----------
    G : NetworkX graph

    weight : string or function
       If this is a string, then edge weights will be accessed via the
       edge attribute with this key (that is, the weight of the edge
       joining `u` to `v` will be ``G.edges[u, v][weight]``). If no
       such edge attribute exists, the weight of the edge is assumed to
       be one.

       If this is a function, the weight of an edge is the value
       returned by the function. The function must accept exactly three
       positional arguments: the two endpoints of an edge and the
       dictionary of edge attributes for that edge. The function must
       return a number.

    Returns
    -------
    negative_cycle : bool
        True if a negative edge cycle exists, otherwise False.

    Examples
    --------
    >>> import networkx as nx
    >>> G = nx.cycle_graph(5, create_using = nx.DiGraph())
    >>> print(nx.negative_edge_cycle(G))
    False
    >>> G[1][2]['weight'] = -7
    >>> print(nx.negative_edge_cycle(G))
    True

    Notes
    -----
    Edge weight attributes must be numerical.
    Distances are calculated as sums of weighted edges traversed.

    This algorithm uses bellman_ford_predecessor_and_distance() but finds
    negative cycles on any component by first adding a new node connected to
    every node, and starting bellman_ford_predecessor_and_distance on that
    node.  It then removes that extra node.
    """
    newnode = generate_unique_node()
    G.add_edges_from([(newnode, n) for n in G])

    try:
        bellman_ford_predecessor_and_distance(G, newnode, weight)
    except nx.NetworkXUnbounded:
        return True
    finally:
        G.remove_node(newnode)
    return False


def bidirectional_dijkstra(G, source, target, weight='weight'):
    """Dijkstra's algorithm for shortest paths using bidirectional search.

    Parameters
    ----------
    G : NetworkX graph

    source : node
       Starting node.

    target : node
       Ending node.

    weight : string or function
       If this is a string, then edge weights will be accessed via the
       edge attribute with this key (that is, the weight of the edge
       joining `u` to `v` will be ``G.edges[u, v][weight]``). If no
       such edge attribute exists, the weight of the edge is assumed to
       be one.

       If this is a function, the weight of an edge is the value
       returned by the function. The function must accept exactly three
       positional arguments: the two endpoints of an edge and the
       dictionary of edge attributes for that edge. The function must
       return a number.

    Returns
    -------
    length, path : number and list
       length is the distance from source to target.
       path is a list of nodes on a path from source to target.

    Raises
    ------
    NodeNotFound
        If either `source` or `target` is not in `G`.

    NetworkXNoPath
        If no path exists between source and target.

    Examples
    --------
    >>> G = nx.path_graph(5)
    >>> length, path = nx.bidirectional_dijkstra(G, 0, 4)
    >>> print(length)
    4
    >>> print(path)
    [0, 1, 2, 3, 4]

    Notes
    -----
    Edge weight attributes must be numerical.
    Distances are calculated as sums of weighted edges traversed.

    In practice  bidirectional Dijkstra is much more than twice as fast as
    ordinary Dijkstra.

    Ordinary Dijkstra expands nodes in a sphere-like manner from the
    source. The radius of this sphere will eventually be the length
    of the shortest path. Bidirectional Dijkstra will expand nodes
    from both the source and the target, making two spheres of half
    this radius. Volume of the first sphere is `\pi*r*r` while the
    others are `2*\pi*r/2*r/2`, making up half the volume.

    This algorithm is not guaranteed to work if edge weights
    are negative or are floating point numbers
    (overflows and roundoff errors can cause problems).

    See Also
    --------
    shortest_path
    shortest_path_length
    """
    if source not in G or target not in G:
        msg = 'Either source {} or target {} is not in G'
        raise nx.NodeNotFound(msg.format(source, target))

    if source == target:
        return (0, [source])
    push = heappush
    pop = heappop
    # Init:  [Forward, Backward]
    dists = [{}, {}]   # dictionary of final distances
    paths = [{source: [source]}, {target: [target]}]  # dictionary of paths
    fringe = [[], []]  # heap of (distance, node) for choosing node to expand
    seen = [{source: 0}, {target: 0}]  # dict of distances to seen nodes
    c = count()
    # initialize fringe heap
    push(fringe[0], (0, next(c), source))
    push(fringe[1], (0, next(c), target))
    # neighs for extracting correct neighbor information
    if G.is_directed():
        neighs = [G.successors, G.predecessors]
    else:
        neighs = [G.neighbors, G.neighbors]
    # variables to hold shortest discovered path
    # finaldist = 1e30000
    finalpath = []
    dir = 1
    while fringe[0] and fringe[1]:
        # choose direction
        # dir == 0 is forward direction and dir == 1 is back
        dir = 1 - dir
        # extract closest to expand
        (dist, _, v) = pop(fringe[dir])
        if v in dists[dir]:
            # Shortest path to v has already been found
            continue
        # update distance
        dists[dir][v] = dist  # equal to seen[dir][v]
        if v in dists[1 - dir]:
            # if we have scanned v in both directions we are done
            # we have now discovered the shortest path
            return (finaldist, finalpath)

        for w in neighs[dir](v):
            if(dir == 0):  # forward
                if G.is_multigraph():
                    minweight = min((dd.get(weight, 1)
                                     for k, dd in G[v][w].items()))
                else:
                    minweight = G[v][w].get(weight, 1)
                vwLength = dists[dir][v] + minweight  # G[v][w].get(weight,1)
            else:  # back, must remember to change v,w->w,v
                if G.is_multigraph():
                    minweight = min((dd.get(weight, 1)
                                     for k, dd in G[w][v].items()))
                else:
                    minweight = G[w][v].get(weight, 1)
                vwLength = dists[dir][v] + minweight  # G[w][v].get(weight,1)

            if w in dists[dir]:
                if vwLength < dists[dir][w]:
                    raise ValueError(
                        "Contradictory paths found: negative weights?")
            elif w not in seen[dir] or vwLength < seen[dir][w]:
                # relaxing
                seen[dir][w] = vwLength
                push(fringe[dir], (vwLength, next(c), w))
                paths[dir][w] = paths[dir][v] + [w]
                if w in seen[0] and w in seen[1]:
                    # see if this path is better than than the already
                    # discovered shortest path
                    totaldist = seen[0][w] + seen[1][w]
                    if finalpath == [] or finaldist > totaldist:
                        finaldist = totaldist
                        revpath = paths[1][w][:]
                        revpath.reverse()
                        finalpath = paths[0][w] + revpath[1:]
    raise nx.NetworkXNoPath("No path between %s and %s." % (source, target))


def johnson(G, weight='weight'):
    r"""Uses Johnson's Algorithm to compute shortest paths.

    Johnson's Algorithm finds a shortest path between each pair of
    nodes in a weighted graph even if negative weights are present.

    Parameters
    ----------
    G : NetworkX graph

    weight : string or function
       If this is a string, then edge weights will be accessed via the
       edge attribute with this key (that is, the weight of the edge
       joining `u` to `v` will be ``G.edges[u, v][weight]``). If no
       such edge attribute exists, the weight of the edge is assumed to
       be one.

       If this is a function, the weight of an edge is the value
       returned by the function. The function must accept exactly three
       positional arguments: the two endpoints of an edge and the
       dictionary of edge attributes for that edge. The function must
       return a number.

    Returns
    -------
    distance : dictionary
       Dictionary, keyed by source and target, of shortest paths.

    Raises
    ------
    NetworkXError
       If given graph is not weighted.

    Examples
    --------
    >>> import networkx as nx
    >>> graph = nx.DiGraph()
    >>> graph.add_weighted_edges_from([('0', '3', 3), ('0', '1', -5),
    ... ('0', '2', 2), ('1', '2', 4), ('2', '3', 1)])
    >>> paths = nx.johnson(graph, weight='weight')
    >>> paths['0']['2']
    ['0', '1', '2']

    Notes
    -----
    Johnson's algorithm is suitable even for graphs with negative weights. It
    works by using the Bellman–Ford algorithm to compute a transformation of
    the input graph that removes all negative weights, allowing Dijkstra's
    algorithm to be used on the transformed graph.

    The time complexity of this algorithm is $O(n^2 \log n + n m)$,
    where $n$ is the number of nodes and $m$ the number of edges in the
    graph. For dense graphs, this may be faster than the Floyd–Warshall
    algorithm.

    See Also
    --------
    floyd_warshall_predecessor_and_distance
    floyd_warshall_numpy
    all_pairs_shortest_path
    all_pairs_shortest_path_length
    all_pairs_dijkstra_path
    bellman_ford_predecessor_and_distance
    all_pairs_bellman_ford_path
    all_pairs_bellman_ford_path_length

    """
    if not nx.is_weighted(G, weight=weight):
        raise nx.NetworkXError('Graph is not weighted.')

    dist = {v: 0 for v in G}
    pred = {v: [] for v in G}
    weight = _weight_function(G, weight)

    # Calculate distance of shortest paths
    dist_bellman = _bellman_ford(G, list(G), weight, pred=pred, dist=dist)

    # Update the weight function to take into account the Bellman--Ford
    # relaxation distances.
    def new_weight(u, v, d):
        return weight(u, v, d) + dist_bellman[u] - dist_bellman[v]

    def dist_path(v):
        paths = {v: [v]}
        _dijkstra(G, v, new_weight, paths=paths)
        return paths

    return {v: dist_path(v) for v in G}

#!/usr/bin/env python

import math
import numpy as np
import rospy
from visualization_msgs.msg import Marker


def normalize_from_minmax(v, min_v, max_v):
    """Normalize a value given their max and minimum values of the lists

    :param v: value to normalize
    :param max_v: maximum value of the list
    :param min_v: minimum value fo the list
    :return:
    """
    # todo why ((10 - 1) + 1)?
    if min_v == 0 and max_v == 0:
        return 0

    return (v - min_v) / float(max_v - min_v)


def create_marker(pos, orientation=1.0, color=(1.0, 1.0, 1.0), m_scale=0.5, frame_id="/os1_init", duration=10,
                  marker_id=0, mesh_resource=None, marker_type=2, marker_text=""):
    """Create marker object using the map information and the node position

    :param pos: list of 3d postion for the marker
    :param orientation: orientation of the maker (1 for no orientation)
    :param color: a 3 vector of 0-1 rgb values
    :param m_scale: scale of the marker (1.0) for normal scale
    :param frame_id: ROS frame id
    :param duration: duration in seconds for this marker dissapearance
    :param marker_id:
    :param mesh_resource:
    :param marker_type: one of the following types (use the int value)
            http://wiki.ros.org/rviz/DisplayTypes/Marker
            ARROW = 0
            CUBE = 1
            SPHERE = 2
            CYLINDER = 3
            LINE_STRIP = 4
            LINE_LIST = 5
            CUBE_LIST = 6
            SPHERE_LIST = 7
            POINTS = 8
            TEXT_VIEW_FACING = 9
            MESH_RESOURCE = 10
            TRIANGLE_LIST = 11
    :param marker_text: text string used for the marker
    :return:
    """

    marker = Marker()
    marker.header.frame_id = frame_id
    marker.id = marker_id

    if mesh_resource:
        marker.type = marker.MESH_RESOURCE
        marker.mesh_resource = mesh_resource
    else:
        marker.type = marker_type

    marker.action = marker.ADD
    marker.scale.x = m_scale
    marker.scale.y = m_scale
    marker.scale.z = m_scale
    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.pose.orientation.w = orientation

    marker.text = marker_text

    marker.pose.position.x = pos[0]
    marker.pose.position.y = pos[1]
    marker.pose.position.z = pos[2]

    d = rospy.Duration.from_sec(duration)
    marker.lifetime = d

    return marker


def angle_between_vectors(v1, v2):
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


def find_closer_centroid(centroids, p, tol=1.e-2, force_return_closer=False):
    """
    Find the centroid closer to P with a tolerance
    :param centroids: list of 3D points
    :param p: 3D point (x ,y, z)
    :param tol: tolerance
    :return:
    """

    source_face = -1
    min_dist = 999999999999

    rospy.loginfo("p:%s centroid_size:%d", p, len(centroids))

    for idx, face_pt in enumerate(centroids):
        d = math.sqrt((face_pt[0] - p[0]) ** 2 + (face_pt[1] - p[1]) ** 2 + (face_pt[2] - p[2]) ** 2)

        # if force is True then return the closest face instead of the
        # best face point considering the tolerance parameter "tol"
        if d < min_dist:
            min_dist = d
            if force_return_closer:
                source_face = idx
                #rospy.loginfo("min_dist: %f", min_dist)

        if np.isclose(face_pt[0], p[0], rtol=tol, atol=tol) and \
            np.isclose(face_pt[1], p[1], rtol=tol, atol=tol) and \
                np.isclose(face_pt[2], p[2], rtol=tol, atol=tol):

            min_dist = d
            source_face = idx
            rospy.loginfo("find direct tolerance: %f", min_dist)
            break

    rospy.loginfo("Returned face idx:%f min_dist:%s", source_face, min_dist)
    return source_face


def energy_weight_non_rot(self, node_source, node_target, centroids):
    """Function from Filipe Rocha's original code to estimate the energy cost
    between two faces

    :param self:
    :param node_source:
    :param node_target:
    :return:
    """
    u = 1
    m = 20  # kg
    g = 9.8  # m/s^2
    points_source = centroids[node_source]
    points_target = centroids[node_target]
    vector2_faces = [points_target[0] - points_source[0], points_target[1] - points_source[1],
                    points_target[2] - points_source[2]]
    vector_xy = [0, 0, -1]
    angle = angle_between_vectors(vector2_faces, vector_xy)
    dist = np.linalg.norm(np.asarray(points_source) - np.asarray(points_target))
    energy = math.fabs((u * m * g * math.cos(angle)) + (m * g * math.sin(angle))) * dist
    return energy

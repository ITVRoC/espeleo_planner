#!/usr/bin/env python

import os
import sys
import rospy
import pymesh
import rospkg
import traceback
from visualization_msgs.msg import Marker
import sensor_msgs.msg
import sensor_msgs.point_cloud2 as pc2
from scipy import spatial
from sklearn.cluster import DBSCAN
import numpy as np
import matplotlib.pyplot as plt
from mayavi import mlab

lidar_msg = None

import numpy as np
from scipy.spatial import distance
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.axes3d import get_test_data
from mpl_toolkits.mplot3d import Axes3D
import math
from math import pi
from recon_surface.srv import MeshFromPointCloud2


def normaldefinition_3D_real(void_data, k):
    """

    This function calculate the normal of a point taking into account the points surrounding it by
    calculating containing them and performing a PCA (Principal Component Analysis)

    INPUT:
        ellipsoid_data: M x 3 array containing the x,y and z coordinates (M is the number of points)
        k: number of neighbor points used to calculate the tnagent plane associated with each point (KNN)

    OUTPUT:
        normals: M x 6 array containing the coordinates of the normal of the plane associated to each point

    """

    " Calculate the K-nearest points to each point (KNN) "

    dist = distance.squareform(
        distance.pdist(void_data))  # Alternative (and more direct) form to calculate the distances
    closest = np.argsort(dist, axis=1)  # Axis = 1 because we are sorting the columns

    "Extraction of normals and centroids for each point"

    total_pts = np.size(closest, 0)
    planes = np.zeros((total_pts,
                       6))  # The three first columns contian the coordinates of the normals. The 3 last columns contain the coordinates of the centroid of the plane

    fig1=plt.figure()
    ax = fig1.add_subplot(1,1,1, projection='3d')
    ax.scatter(void_data[:, 0], void_data[:, 1], void_data[:, 2], facecolors='none',edgecolors='r')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    for i in range(total_pts):
        normal_vect, xmn, ymn, zmn, knn_pt_coord = tangentplane_3D_real(closest[i, :], void_data,
                                                                        k)  # Obtention of the normal and centroid (and other parametres) for each point in the ellipsoid

        planes[i, 0:3] = normal_vect  # Keep the coordinates of the normal vectors
        planes[i, 3:6] = np.array([xmn, ymn, zmn])  # Keep the coordinates of the centroid

        ax.scatter(xmn,ymn,zmn, facecolors='none',edgecolors = 'c')
        ax.plot([xmn,xmn+normal_vect[0]],[ymn,ymn+normal_vect[1]],[zmn,zmn+normal_vect[2]])

    planes_consist = normalconsistency_3D_real(planes)

    plt.show()
    plt.hold(True)

    return planes, planes_consist


def tangentplane_3D_real(closest_pt, ellipsoid_data, k):
    """

    This function calculates the centre point c of a cloud of points knni and returns
    the normal of the tangent plane best fitting the coud of points

        Input:
            knni: Array of indexes representing the nearest point (index) to the first point of the row knni[i]
            ellipsoid: x,y,z coordinates of points defining the ellipsoid. We will use it to give coordinates to each index in knni
            k: Number of closest points we want to calculate for a given point

    """

    "Extraction of the coordinates of each point from their indexes and the matrix ellipsoid"

    knn_pt_id = closest_pt[0:k]  # Retain only the indexes of the k-closest points
    nb_points = np.size(knn_pt_id)
    knn_pt_coord = np.zeros((nb_points, 3))

    for i in range(nb_points):
        point_i = knn_pt_id[i]

        knn_pt_coord[i, :] = ellipsoid_data[point_i, :]

    "Principal component analysis (PCA)"

    "Centorid calculation"

    xmn = np.mean(knn_pt_coord[:, 0])
    ymn = np.mean(knn_pt_coord[:, 1])
    zmn = np.mean(knn_pt_coord[:, 2])

    c = np.zeros((np.size(knn_pt_coord, 0), 3))

    c[:, 0] = knn_pt_coord[:, 0] - xmn
    c[:, 1] = knn_pt_coord[:, 1] - ymn
    c[:, 2] = knn_pt_coord[:, 2] - zmn

    "Covariance matrix"

    cov = np.zeros((3, 3))

    cov[0, 0] = np.dot(c[:, 0], c[:, 0])
    cov[0, 1] = np.dot(c[:, 0], c[:, 1])
    cov[0, 2] = np.dot(c[:, 0], c[:, 2])

    cov[1, 0] = cov[0, 1]
    cov[1, 1] = np.dot(c[:, 1], c[:, 1])
    cov[1, 2] = np.dot(c[:, 1], c[:, 2])

    cov[2, 0] = cov[0, 2]
    cov[2, 1] = cov[1, 2]
    cov[2, 2] = np.dot(c[:, 2], c[:, 2])

    "Single value decomposition (SVD)"

    u, s, vh = np.linalg.svd(cov)  # U contains the orthonormal eigenvectors and S contains the eigenvectors

    "Selection of minimum eigenvalue"

    minevindex = np.argmin(s)

    "Selection of orthogonal vector corresponing to this eigenvalue --> vector normal to the plane defined by the kpoints"

    normal_vect = u[:, minevindex]

    return normal_vect, xmn, ymn, zmn, knn_pt_coord


def normalconsistency_3D_real(planes):
    """

    This function checks wherer the normals are oriented towards the outside of the surface, i.e., it
    checks the consistency of the normals.
    The function changes the direction of the normals that do not point towards the outside of the shape
    The function checks whether the normals are oriented towards the centre of the ellipsoid,
    and if YES, then, it turns their orientation

    INPUTS:
        planes: Vector N x 6, where M is the number of points whose normals and
        centroid have been calculated. the columns are the coordinates of the normals and the centroids

    OUTPUTS:
        planesconsist: N x 6 array, where N is the number of points whose planes have been calculated. This array
        has all the planes normals pointing outside the surface.

    """

    nbnormals = np.size(planes, 0)
    planes_consist = np.zeros((nbnormals, 6))
    planes_consist[:, 3:6] = planes[:,
                             3:6]  # We just copy the columns corresponding to the coordinates of the centroids (from 3th to 5th)

    """ Try the atan2 function : https://uk.mathworks.com/help/vision/ref/pcnormals.html#buxdmoj"""

    sensorcentre = np.array([0, 0, 0])

    for i in range(nbnormals):

        p1 = (sensorcentre - planes[i, 3:6]) / np.linalg.norm(sensorcentre - planes[i,
                                                                             3:6])  # Vector from the centroid to the centre of the ellipsoid (here the sensor is placed)
        p2 = planes[i, 0:3]

        angle = math.atan2(np.linalg.norm(np.cross(p1, p2)),
                           np.dot(p1, p2))  # Angle between the centroid-sensor and plane normal

        if (angle >= -pi / 2 and angle <= pi / 2):  # (angle >= -pi/2 and angle <= pi/2):

            planes_consist[i, 0] = -planes[i, 0]
            planes_consist[i, 1] = -planes[i, 1]
            planes_consist[i, 2] = -planes[i, 2]

        else:

            planes_consist[i, 0] = planes[i, 0]
            planes_consist[i, 1] = planes[i, 1]
            planes_consist[i, 2] = planes[i, 2]

    return planes_consist


def get_centroid_of_pts(arr):
    length = arr.shape[0]
    sum_x = np.sum(arr[:, 0])
    sum_y = np.sum(arr[:, 1])
    sum_z = np.sum(arr[:, 2])
    return np.array([[sum_x/length, sum_y/length, sum_z/length]])


def lidar_callback(msg):
    global lidar_msg
    #rospy.loginfo("lidar_callback")
    lidar_msg = msg


def process_lidar_msg():
    global lidar_msg

    if not lidar_msg:
        return

    mlab.figure(1, bgcolor=(0, 0, 0))
    mlab.clf()

    points = pc2.read_points_list(lidar_msg, field_names=("x", "y", "z"), skip_nans=True)

    print "size points:", len(points)
    points = [p for p in points if p[2] < -0.39 or p[2] > -0.35 and math.sqrt(p[0] ** 2 + p[1] ** 2 + p[2] ** 2) < 3.0]
    print "size points after Z basic cleanout:", len(points)

    X = np.array(points)
    pts2 = mlab.points3d(X[:, 0], X[:, 1], X[:, 2],
                         color=(1.0, 1.0, 1.0),
                         scale_factor=0.01,
                         scale_mode='none',
                         resolution=20)


    border_kdtree = spatial.KDTree(points)
    border_tresh = 0.4

    db = DBSCAN(eps=0.20, min_samples=2).fit(points)
    core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
    core_samples_mask[db.core_sample_indices_] = True
    labels = db.labels_

    # Number of clusters in labels, ignoring noise if present.
    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
    n_noise_ = list(labels).count(-1)

    rospy.loginfo('Estimated number of clusters: %d, noise_points: %s', n_clusters_, n_noise_)
    unique_labels = set(labels)
    colors = plt.cm.get_cmap('gist_rainbow', len(unique_labels))

    X = np.array(points)
    for idx, label in enumerate(unique_labels):
        if label == -1:
            # Black used for noise.
            # col = [0, 0, 0, 1]
            continue
        else:
            col = colors(idx)

        # print "col", col

        class_member_mask = (labels == label)
        xyz = X[class_member_mask & core_samples_mask]
        # print "xyz:", xyz

        z_std = np.std(xyz[:, 2])
        print "np.std(xyz[:, 2]):", z_std

        #normaldefinition_3D_real(xyz, 16)

        # mesh_filepath = None
        # try:
        #     fields = [
        #         sensor_msgs.msg.PointField('x', 0, sensor_msgs.msg.PointField.FLOAT32, 1),
        #         sensor_msgs.msg.PointField('y', 4, sensor_msgs.msg.PointField.FLOAT32, 1),
        #         sensor_msgs.msg.PointField('z', 8, sensor_msgs.msg.PointField.FLOAT32, 1)
        #     ]
        #
        #     pcloud = pc2.create_cloud(lidar_msg.header, fields, xyz.tolist())
        #     rospy.wait_for_service('/mesh_from_pointclouds', timeout=3)
        #     mesh_from_pointcloud = rospy.ServiceProxy('/mesh_from_pointclouds', MeshFromPointCloud2)
        #     resp1 = mesh_from_pointcloud(pcloud)
        #     mesh_filepath = resp1.path
        #     rospy.loginfo("pointcloud processed result: %s", resp1)
        # except rospy.ServiceException as e:
        #     rospy.logerr("Service call failed: %s", e)
        # except Exception as e:
        #     rospy.logerr("Exception: %s", e)
        #
        # if mesh_filepath is None:
        #     rospy.logerr("mesh_filepath is None, cannot continue with the planning")
        #     rate_slow.sleep()
        #     continue

        # fxyz = np.array([centroids[v] for v in intersecting_frontiers])
        pts2 = mlab.points3d(xyz[:, 0], xyz[:, 1], xyz[:, 2],
                             color=(col[0], col[1], col[2]),
                             scale_factor=0.05,
                             scale_mode='none',
                             resolution=20)

        # if this cluster is not having multiple Z levels discard it
        if z_std < 0.1:
            continue

        centroid = get_centroid_of_pts(xyz)
        # print "centroid:", centroid
        pts3 = mlab.points3d(centroid[:, 0], centroid[:, 1], centroid[:, 2],
                             color=(col[0], col[1], col[2]),
                             scale_factor=0.5,
                             scale_mode='none',
                             resolution=20)

    mlab.show()


if __name__ == '__main__':
    rospy.init_node('obstacle_detection_3d_lidar')

    rospy.Subscriber('/velodyne/points2', sensor_msgs.msg.PointCloud2, lidar_callback)
    rate_slow = rospy.Rate(0.1)

    while not rospy.is_shutdown():
        try:
            process_lidar_msg()
        except Exception as e:
            tb = traceback.format_exc()
            rospy.logerr("Main Exception: %s", str(tb))

        rate_slow.sleep()

    rospy.loginfo("obstacle_detection_3d_lidar node stop")

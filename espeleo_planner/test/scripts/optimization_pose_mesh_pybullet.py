#!/usr/bin/env python

import sys
import os
import math
import pybullet as p
import pybullet
import networkx as nx
import numpy as np
import time
import pybullet_data
import pyquaternion
import datetime
from matplotlib import pyplot
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d import Axes3D


def Rxyz(x, y, z, order="ZXY"):
    """
    # Convert Euler Angles passed in a vector of Radians
    # into a rotation matrix.  The individual Euler Angles are
    # processed in the order requested.
    # https://stackoverflow.com/questions/1568568/how-to-convert-euler-angles-to-directional-vector
    :param x:
    :param y:
    :param z:
    :param order:
    :return:
    """

    Sx = math.sin(x)
    Sy = math.sin(y)
    Sz = math.sin(z)
    Cx = math.cos(x)
    Cy = math.cos(y)
    Cz = math.cos(z)

    if order == "XYZ":
        return np.array([[Cy*Cz, -Cy*Sz, Sy],
                         [Cz*Sx*Sy+Cx*Sz, Cx*Cz-Sx*Sy*Sz, -Cy*Sx],
                         [-Cx*Cz*Sy+Sx*Sz, Cz*Sx+Cx*Sy*Sz, Cx*Cy]])

    elif order == "YZX":
        return np.array([[Cy*Cz, Sx*Sy-Cx*Cy*Sz, Cx*Sy+Cy*Sx*Sz],
                        [Sz, Cx*Cz, -Cz*Sx],
                        [-Cz*Sy, Cy*Sx+Cx*Sy*Sz, Cx*Cy-Sx*Sy*Sz]])

    elif order == "ZXY":
        return np.array([[Cy*Cz-Sx*Sy*Sz, -Cx*Sz, Cz*Sy+Cy*Sx*Sz],
                        [Cz*Sx*Sy+Cy*Sz, Cx*Cz, -Cy*Cz*Sx+Sy*Sz],
                        [-Cx*Sy, Sx, Cx*Cy]])

    elif order == "ZYX":
        return np.array([[Cy*Cz, Cz*Sx*Sy-Cx*Sz, Cx*Cz*Sy+Sx*Sz],
                        [Cy*Sz, Cx*Cz+Sx*Sy*Sz, -Cz*Sx+Cx*Sy*Sz],
                        [-Sy, Cy*Sx, Cx*Cy]])

    elif order == "YXZ":
        return np.array([[Cy*Cz+Sx*Sy*Sz, Cz*Sx*Sy-Cy*Sz, Cx*Sy],
                        [Cx*Sz, Cx*Cz, -Sx],
                        [-Cz*Sy+Cy*Sx*Sz, Cy*Cz*Sx+Sy*Sz, Cx*Cy]])

    elif order == "YXZ":
        return np.array([[Cy*Cz, -Sz, Cz*Sy],
                         [Sx*Sy+Cx*Cy*Sz, Cx*Cz, -Cy*Sx+Cx*Sy*Sz],
                         [-Cx*Sy+Cy*Sx*Sz, Cz*Sx, Cx*Cy+Sx*Sy*Sz]])
    else:
        raise ValueError("Order '{}' does not match any known order".format(order))


physicsClient = p.connect(p.GUI)
#physicsClient = p.connect(p.DIRECT)
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0, 0, -9.8)
dt = 1e-3
p.setTimeStep(dt)

col_shape_id = p.createCollisionShape(
    shapeType=pybullet.GEOM_MESH,
    fileName="/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/test/maps/map_01_frontiers.stl",
    flags=p.GEOM_FORCE_CONCAVE_TRIMESH #flags=p.URDF_INITIALIZE_SAT_FEATURES #|p.GEOM_FORCE_CONCAVE_TRIMESH should only be used with fixed (mass=0) objects!
)

viz_shape_id = p.createVisualShape(
    shapeType=p.GEOM_MESH,
    fileName="/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/test/maps/map_01_frontiers.stl",
    rgbaColor=(0.6, 0.3, 0.1, 0.7)
)

body_id = p.createMultiBody(
    baseCollisionShapeIndex=col_shape_id,
    baseVisualShapeIndex=viz_shape_id,
    basePosition=(0, 0, 0),
    baseOrientation=(0, 0, 0, 1),
)

cam_dist = 2.60
cam_yaw = 97.60
cam_pitch = -6.20
cam_pos = [-0.03, 0.24, 0]
p.resetDebugVisualizerCamera( cameraDistance=cam_dist, cameraYaw=cam_yaw, cameraPitch=cam_pitch, cameraTargetPosition=cam_pos)


def estimate_position(startPos, startOrientation):
    start_time = datetime.datetime.now()
    startQuatOrientation = p.getQuaternionFromEuler(startOrientation)

    # boxId = p.loadURDF("/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/urdf/espeleo2_low_poly.urdf",startPos, startQuatOrientation)
    # linkId = 0

    boxId = p.loadURDF(
        "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/urdf/espeleo2_low_poly_prismatic.urdf",
        startPos, startQuatOrientation)
    linkId = 1

    jointFrictionForce = 0
    for joint in range(p.getNumJoints(boxId)):
        p.setJointMotorControl2(boxId, joint, p.POSITION_CONTROL, force=jointFrictionForce)

    #print " ------------- ", p.getNumBodies()

    # prevCubePos, prevCubeOrn = p.getBasePositionAndOrientation(boxId)

    linkStateFull = p.getLinkState(boxId, linkId)
    prevCubePos, prevCubeOrn = linkStateFull[4], linkStateFull[5]

    for i in range(1000):
        p.stepSimulation()

        #print " ------------- 0 ", p.getLinkState(boxId, 0)
        # print " ------------- 1 ", p.getLinkState(boxId, 1)
        # print " ------------- 2 ", p.getLinkState(boxId, 2)
        # print " ------------- 3 ", p.getLinkState(boxId, 3)
        # print " ------------- 4 ", p.getLinkState(boxId, 4)
        # print " ------------- 5 ", p.getLinkState(boxId, 5)
        # print " ------------- 6 ", p.getLinkState(boxId, 6)
        #multiplyTransforms

        #print "------- p.getLinkState():", p.getLinkState(boxId, 0)
        #print "------- p.getDynamicsInfo:", p.getDynamicsInfo(boxId, -1)

        #cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)

        linkStateFull = p.getLinkState(boxId, linkId)
        cubePos, cubeOrn = linkStateFull[4], linkStateFull[5]

        #print "cubePos:", cubePos, cubeOrn

        # dist = math.sqrt((cubePos[0] - prevCubePos[0])**2 +
        #                  (cubePos[1] - prevCubePos[1])**2 +
        #                  (cubePos[2] - prevCubePos[2])**2)

        dist = math.sqrt((cubePos[2] - prevCubePos[2]) ** 2) # only z

        q0 = pyquaternion.Quaternion(cubeOrn)
        q1 = pyquaternion.Quaternion(prevCubeOrn)
        quat_dist = pyquaternion.Quaternion.absolute_distance(q0, q1)

        #print "dist:{:.8f}".format(dist), "quat_dist:{:.6f}".format(quat_dist)

        prevCubePos, prevCubeOrn = cubePos, cubeOrn

        if i > 10 and (dist <= 0.00001 and quat_dist <= 0.0001):
            print "breaking at step:", i, dist, quat_dist
            break

        #time.sleep(1./360.)  # faster
        #time.sleep(1. / 120.)  # slower
        time.sleep(1. / 720.)  # fast

        # # bv = p.getBaseVelocity(boxId)
        # # print "base_vel:", bv
        # # bv = (0.0, 0.0, bv[0][2] + 0.5, bv[1][0], bv[1][1], bv[1][2])
        # # p.resetBaseVelocity(boxId, bv)

    end_time = datetime.datetime.now()
    delta = end_time - start_time
    delta_millis = int(delta.total_seconds() * 1000)  # milliseconds
    print "total time millis:{}".format(delta_millis)

    p.removeBody(boxId)

    print "euler from quat:", p.getEulerFromQuaternion(prevCubeOrn)
    print "q1.vector:", q1.vector
    print "q1.degrees:", q1.degrees

    #R = q1.rotation_matrix
    R = Rxyz(p.getEulerFromQuaternion(prevCubeOrn)[0], p.getEulerFromQuaternion(prevCubeOrn)[1], p.getEulerFromQuaternion(prevCubeOrn)[2])  # , order="YZX"


    # PLOT POLYGON
    print "R:", R

    xf = np.array([1, 0, 0])
    yf = np.array([0, 1, 0])
    zf = np.array([0, 0, 1])
    zf_l = np.dot(R, zf)

    print "q1R.vector:", (zf_l[0], zf_l[1], zf_l[2])

    print "pos:", prevCubePos, "orn:", prevCubeOrn

    dx = 0.212  # distancia entre p1 e p2 = distancia entre p2 e p3 ...
    dy = 0.33  # distancia entre p6 e p1 = distancia entre p4 e p3
    dy_m = 0.425  # distancia entre p5 e p2 - rodas mais afastadas

    poly = np.array([
        [dx, -(dy / 2), 0],
        [0, -(dy_m / 2), 0],
        [-dx, -(dy / 2), 0],
        [-dx, (dy / 2), 0],
        [0, (dy_m / 2), 0],
        [dx, (dy / 2), 0]
    ])

    fig = pyplot.figure()  # figsize=pyplot.figaspect(0.5)*1.1
    # ax = fig.axes(projection="3d")
    ax = Axes3D(fig)

    local_poly = np.dot(poly, R.T)

    ax.scatter(local_poly[:, 0], local_poly[:, 1], local_poly[:, 2], zdir='z', c='b')

    stability_poly_tuples = list([map(list, local_poly)])
    collection = Poly3DCollection(list(stability_poly_tuples), linewidths=0.5, alpha=0.7, edgecolors='blue')
    face_color = [0.5, 0.5, 1]  # alternative: matplotlib.colors.rgb2hex([0.5, 0.5, 1])
    collection.set_facecolor(face_color)
    ax.add_collection3d(collection)

    cz = np.mean(local_poly[:, 2])  # mean of z values

    # center = np.array([origin[0], origin[1], cz + 0.2])
    xf = np.array([1, 0, 0])
    yf = np.array([0, 1, 0])
    zf = np.array([0, 0, 1])
    xf_l = np.dot(R, xf)
    yf_l = np.dot(R, yf)
    zf_l = np.dot(R, zf)

    ax.quiver(0, 0, cz, zf_l[0], zf_l[1], zf_l[2], length=0.2,
              pivot='tail', linestyle="-", color='blue')  # z

    def axisEqual3D(ax):
        extents = np.array([getattr(ax, 'get_{}lim'.format(dim))() for dim in 'xyz'])
        sz = extents[:, 1] - extents[:, 0]
        centers = np.mean(extents, axis=1)
        maxsize = max(abs(sz))
        r = maxsize / 2
        for ctr, dim in zip(centers, 'xyz'):
            getattr(ax, 'set_{}lim'.format(dim))(ctr - r, ctr + r)

    axisEqual3D(ax)

    # scaling = np.array([getattr(ax, 'get_{}lim'.format(dim))() for dim in 'xyz']);
    # ax.auto_scale_xyz(*[[np.min(scaling), np.max(scaling)]] * 3)

    pyplot.show()

    return prevCubePos, prevCubeOrn


# pos, orn = estimate_position([0, 0, 1], [0, 0, 0])
# #pos, orn = estimate_position([0, 0, 1], [0, 0, 0])
# pos, orn = estimate_position([1, 0.5, 1], [0, 0, 0])
# pos, orn = estimate_position([0, 1.5, 1], [0, 0, 0])
pos, orn = estimate_position([0, 1.8, 1], [0, 0, 0])

print "finishing..."
#time.sleep(4)
p.disconnect()
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

#physicsClient = p.connect(p.GUI)
physicsClient = p.connect(p.DIRECT)
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0, 0, -9.8)
dt = 1e-3
p.setTimeStep(dt)

col_shape_id = p.createCollisionShape(
    shapeType=pybullet.GEOM_MESH,
    fileName="/tmp/tmp.stl",
    flags=p.GEOM_FORCE_CONCAVE_TRIMESH #flags=p.URDF_INITIALIZE_SAT_FEATURES #|p.GEOM_FORCE_CONCAVE_TRIMESH should only be used with fixed (mass=0) objects!
)

viz_shape_id = p.createVisualShape(
    shapeType=p.GEOM_MESH,
    fileName="/tmp/tmp.stl",
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

        #time.sleep(1./360.)

        # # bv = p.getBaseVelocity(boxId)
        # # print "base_vel:", bv
        # # bv = (0.0, 0.0, bv[0][2] + 0.5, bv[1][0], bv[1][1], bv[1][2])
        # # p.resetBaseVelocity(boxId, bv)

    end_time = datetime.datetime.now()
    delta = end_time - start_time
    delta_millis = int(delta.total_seconds() * 1000)  # milliseconds
    print "total time millis:{}".format(delta_millis)

    p.removeBody(boxId)

    return prevCubePos, prevCubeOrn


pos, orn = estimate_position([0, 0, 1], [0, 0, 0])
#pos, orn = estimate_position([0, 0, 1], [0, 0, 0])
pos, orn = estimate_position([1, 0.5, 1], [0, 0, 0])

print "finishing..."
time.sleep(4)
p.disconnect()
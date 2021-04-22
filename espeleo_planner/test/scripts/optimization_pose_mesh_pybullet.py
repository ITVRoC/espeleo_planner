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

physicsClient = p.connect(p.GUI)
#physicsClient = p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0, 0, -10)

col_shape_id = p.createCollisionShape(
    shapeType=pybullet.GEOM_MESH,
    fileName="/tmp/tmp.stl",
    flags=p.GEOM_FORCE_CONCAVE_TRIMESH #flags=p.URDF_INITIALIZE_SAT_FEATURES #|p.GEOM_FORCE_CONCAVE_TRIMESH should only be used with fixed (mass=0) objects!
)

viz_shape_id = p.createVisualShape(
    shapeType=p.GEOM_MESH,
    fileName="/tmp/tmp.stl",
)

body_id = p.createMultiBody(
    baseCollisionShapeIndex=col_shape_id,
    baseVisualShapeIndex=viz_shape_id,
    basePosition=(0, 0, 0),
    baseOrientation=(0, 0, 0, 1),
)


def estimate_position(startPos, startOrientation):
    start_time = datetime.datetime.now()
    startQuatOrientation = p.getQuaternionFromEuler(startOrientation)

    boxId = p.loadURDF("/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/urdf/espeleo2_low_poly.urdf",startPos, startQuatOrientation)

    prevCubePos, prevCubeOrn = p.getBasePositionAndOrientation(boxId)
    for i in range(1000):
        p.stepSimulation()
        cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)

        dist = math.sqrt((cubePos[0] - prevCubePos[0])**2 +
                         (cubePos[1] - prevCubePos[1])**2 +
                         (cubePos[2] - prevCubePos[2])**2)

        q0 = pyquaternion.Quaternion(cubeOrn)
        q1 = pyquaternion.Quaternion(prevCubeOrn)
        quat_dist = pyquaternion.Quaternion.absolute_distance(q0, q1)

        #print "dist:{:.6f}".format(dist), "quat_dist:{:.6f}".format(quat_dist)

        prevCubePos, prevCubeOrn = cubePos, cubeOrn

        if i > 5 and (dist <= 0.001 and quat_dist <= 0.001):
            print "breaking at step:", i
            break

        time.sleep(1./360.)

    end_time = datetime.datetime.now()
    delta = end_time - start_time
    delta_millis = int(delta.total_seconds() * 1000)  # milliseconds
    print "total time millis:{}".format(delta_millis)

    p.removeBody(boxId)

    return prevCubePos, prevCubeOrn


pos, orn = estimate_position([0, 0, 2], [0, 0, 0])
#pos, orn = estimate_position([0, 0, 2], [0, 0, 0])
pos, orn = estimate_position([1, 1, 2], [0, 0, 0])

print "finishing..."
time.sleep(4)
p.disconnect()
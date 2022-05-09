#!/usr/bin/env python

import math
import pybullet as p
import pybullet
import pyquaternion
import datetime
import rospkg
import os
from . import mesh_helper
import numpy as np
import time


class PybulletAngleEstimation:

    def __init__(self, mesh_path, show_GUI=False):

        if mesh_path is None or not os.path.isfile(mesh_path):
            raise ValueError("mesh_path is not valid: {}".format(mesh_path))

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('espeleo_planner')
        self.urdf_path = os.path.join(pkg_path, "urdf", "espeleo2_low_poly_prismatic.urdf")

        if self.urdf_path is None or not os.path.isfile(self.urdf_path):
            raise ValueError("urdf_path is not valid: {}".format(self.urdf_path))

        if show_GUI:
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)

        p.resetSimulation()
        p.setGravity(0, 0, -9.8)
        dt = 1e-3
        p.setTimeStep(dt)

        self.col_shape_id = p.createCollisionShape(
            shapeType=pybullet.GEOM_MESH,
            fileName=mesh_path,
            flags=p.GEOM_FORCE_CONCAVE_TRIMESH
        )

        self.viz_shape_id = p.createVisualShape(
            shapeType=p.GEOM_MESH,
            fileName=mesh_path,
            rgbaColor=(0.6, 0.3, 0.1, 0.7)
        )

        self.body_id = p.createMultiBody(
            baseCollisionShapeIndex=self.col_shape_id,
            baseVisualShapeIndex=self.viz_shape_id,
            basePosition=(0, 0, 0),
            baseOrientation=(0, 0, 0, 1),
        )

    def estimate_pose(self, start_pos, z_distance=2, start_orientation=(0, 0, 0)):
        start_time = datetime.datetime.now()
        start_quat_orientation = p.getQuaternionFromEuler(start_orientation)

        start_pos = [start_pos[0], start_pos[1], start_pos[2] + z_distance]

        boxId = p.loadURDF(
            self.urdf_path,
            start_pos, start_quat_orientation)
        linkId = 1

        jointFrictionForce = 0
        for joint in range(p.getNumJoints(boxId)):
            p.setJointMotorControl2(boxId, joint, p.POSITION_CONTROL, force=jointFrictionForce)

        linkStateFull = p.getLinkState(boxId, linkId)
        prevCubePos, prevCubeOrn = linkStateFull[4], linkStateFull[5]

        for i in range(1000):
            p.stepSimulation()

            linkStateFull = p.getLinkState(boxId, linkId)
            cubePos, cubeOrn = linkStateFull[4], linkStateFull[5]

            dist = math.sqrt((cubePos[2] - prevCubePos[2]) ** 2)  # only z

            q0 = pyquaternion.Quaternion(x=cubeOrn[0], y=cubeOrn[1], z=cubeOrn[2], w=cubeOrn[3])
            q1 = pyquaternion.Quaternion(x=prevCubeOrn[0], y=prevCubeOrn[1], z=prevCubeOrn[2], w=prevCubeOrn[3])
            quat_dist = pyquaternion.Quaternion.absolute_distance(q0, q1)

            # print "dist:{:.8f}".format(dist), "quat_dist:{:.6f}".format(quat_dist)

            prevCubePos, prevCubeOrn = cubePos, cubeOrn

            if i > 10 and (dist <= 0.00001 and quat_dist <= 0.0001):
                #print "breaking at step:", i, dist, quat_dist
                break

            #time.sleep(1./360.)  # slow
            #time.sleep(1./1440.)  # faster

        #time.sleep(1.)

        end_time = datetime.datetime.now()
        delta = end_time - start_time
        delta_millis = int(delta.total_seconds() * 1000)  # milliseconds
        #print "total time millis:{}".format(delta_millis)

        p.removeBody(boxId)

        euler_q1 = p.getEulerFromQuaternion(prevCubeOrn)
        R = mesh_helper.Rxyz(euler_q1[0], euler_q1[1], euler_q1[2])
        zf = np.array([0, 0, 1])
        zf_l = np.dot(R, zf)

        return prevCubePos, zf_l

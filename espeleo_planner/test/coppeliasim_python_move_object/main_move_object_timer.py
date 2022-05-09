# -*- coding: utf-8 -*-

import sim
import time
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm


def show_object(client_id, name):
    print(f"activating: {name}")
    res, obj_handle = sim.simxGetObjectHandle(client_id, name, sim.simx_opmode_oneshot_wait)
    ret, pos = sim.simxGetObjectPosition(client_id, obj_handle, -1, sim.simx_opmode_oneshot_wait)
    
    new_pos = [pos[0], pos[1], pos[2] + 5.0]
    ret = sim.simxSetObjectPosition(client_id, obj_handle, -1, new_pos, sim.simx_opmode_oneshot_wait)

if __name__ == '__main__':
    
    print("Load simulation...")
    sim.simxFinish(-1) # just in case, close all opened connections

    print("simxStart...")
    client_id = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    assert client_id != -1, 'Failed connecting to remote API server'

    print("simxStartSimulation...")
    e = sim.simxStartSimulation(client_id, sim.simx_opmode_oneshot)

    # print ping time
    print("pinging...")
    sec, msec = sim.simxGetPingTime(client_id)
    print("Ping time: %f" % (sec + msec / 1000.0))

    # ret, objectHandles = sim.simxGetObjects(client_id, 0, sim.simx_opmode_oneshot_wait)
    # print(objectHandles)

    error, handlers, intData, floatData, stringData=sim.simxGetObjectGroupData(client_id, 109, 0, sim.simx_opmode_oneshot_wait)
    print(stringData)

    object_names = ['Cuboid0']
    for name in object_names:
        show_object(client_id, name)

    object_names = ['Cuboid1#0']
    input(f"Press enter to show {object_names}")
    for name in object_names:
        show_object(client_id, name)

    # Now send some data to CoppeliaSim in a non-blocking fashion:
    #sim.simxAddStatusbarMessage(client_id, 'Hello CoppeliaSim!', sim.simx_opmode_oneshot)

    sec, msec = sim.simxGetPingTime(client_id)
    print("Ping time: %f" % (sec + msec / 1000.0))

    #print("Finishing client...")
    #sim.simxFinish(client_id)
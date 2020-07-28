#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from recon_surface.cfg import ReconSurfaceConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {}""".format(config))
    return config

if __name__ == "__main__":
    rospy.init_node("recon_surface_dyn_server_py", anonymous = False)

    srv = Server(ReconSurfaceConfig, callback)
    rospy.spin()

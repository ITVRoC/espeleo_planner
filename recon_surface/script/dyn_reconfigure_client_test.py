#!/usr/bin/env python

import rospy

import dynamic_reconfigure.client

def callback(config):
    rospy.loginfo("Config set to: {}".format(config))


if __name__ == "__main__":
    rospy.init_node("surface_recon_dynamic_client")

    client = dynamic_reconfigure.client.Client("recon_dyn_server_node", timeout=5, config_callback=callback)
    is_forward = client.get_configuration
    r = rospy.Rate(1)
    #b = False
    while not rospy.is_shutdown():
        #b = not b
        is_forward = client.get_configuration
        client.update_configuration({"trim": 0.99})
        r.sleep()

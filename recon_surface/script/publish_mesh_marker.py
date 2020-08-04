#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
import rospkg


if __name__ == "__main__":
    rospy.init_node("test_publish_stl_marker", anonymous = False)

    pub = rospy.Publisher("/reconstructed_mesh_marker_normal", Marker, queue_size=1)

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('recon_surface')

    while not rospy.is_shutdown():

        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.MESH_RESOURCE
        marker.mesh_resource = "file:// " + pkg_path + "/test/map_medium_mesh.stl"
        marker.action = marker.ADD

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        pub.publish(marker)

        rospy.sleep(1.0)
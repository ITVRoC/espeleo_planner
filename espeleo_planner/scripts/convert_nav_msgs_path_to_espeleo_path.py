#!/usr/bin/env python

import rospy
from espeleo_control.msg import Path
import std_msgs.msg
from geometry_msgs.msg import Polygon


class PathConverterEspeleo:
    """
    This node converts the normal polygon path to a Espeleorobo path message
    """

    def __init__(self):
        self.path_pub = None
        self.init_node()

    def init_node(self):
        """
        Init node with ROS bindings
        :return:
        """

        self.path_pub = rospy.Publisher('/espeleo/traj_points', Path, queue_size=1)
        rospy.Subscriber('/espeleo/traj_points_polygon', Polygon, self.path_callback)

    def path_callback(self, msg):
        """
        Callback to the polygon messages. It converts polygon to espeleo path msg and publish it
        :param msg: Polygon message
        :return:
        """

        espeleo_path = Path()
        espeleo_path.header = std_msgs.msg.Header()
        espeleo_path.header.stamp = rospy.Time.now()
        espeleo_path.closed_path_flag = False
        espeleo_path.insert_n_points = 5
        espeleo_path.filter_path_n_average = 5
        
        espeleo_path.path = Polygon()
        espeleo_path.path.points = msg.points

        self.path_pub.publish(espeleo_path)


if __name__ == '__main__':
    rospy.init_node('path_converter_espeleo', anonymous=True)
    rospy.loginfo("PathConverterEspeleo node start")

    pconverter = PathConverterEspeleo()

    rospy.spin()

    rospy.loginfo("PathConverterEspeleo node stop")

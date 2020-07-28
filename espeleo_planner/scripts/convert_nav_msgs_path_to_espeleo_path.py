#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep
from visualization_msgs.msg import Marker, MarkerArray
import tf
import sys
from tf2_msgs.msg import TFMessage
import numpy as np
from espeleo_control.msg import Path
import nav_msgs.msg
import std_msgs.msg
from geometry_msgs.msg import Polygon, Point32

import pprint 

pp = pprint.PrettyPrinter(indent=4)


class PathConverterEspeleo:
    """
    Doc here #todo
    """

    def __init__(self):
        self.path_pub = None

        self.init_node()

    def init_node(self):
        rospy.init_node('path_converter_espeleo', anonymous=True)

        # publishers
        self.path_pub = rospy.Publisher('/espeleo/traj_points', Path, queue_size=1)

        # subscribers
        rospy.Subscriber('/espeleo/traj_points_polygon', Polygon, self.path_callback)

    def path_callback(self, msg):

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
    rospy.loginfo("PathConverterEspeleo node start")
    pconverter = PathConverterEspeleo()

    rospy.spin()

    rospy.loginfo("PathConverterEspeleo node stop")

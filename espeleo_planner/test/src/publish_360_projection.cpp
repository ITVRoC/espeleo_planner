#include <iostream>
#include <fstream>
#include <chrono>
#include <algorithm>
#include <iterator>

#include <octomap/octomap.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>
#include <tf/transform_broadcaster.h>
#include <ros/console.h>
#include <octomap/math/Utils.h>
#include <octomap/math/Vector3.h>
#include <map>
#include <limits>
#include "boost/tuple/tuple_comparison.hpp"
#include <cmath>

#include "frontier_entropy.h"

using namespace std;
using namespace std::chrono;

int main(int argc, char **argv) {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::console::notifyLoggerLevelsChanged();

    ros::init(argc, argv, "test_360_projection_pov");
    ros::NodeHandle nh;
    ros::Rate r(0.5);

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray> ("/projection", 0);

    OctomapExploration octomap_exploration(nh);

    // only 180 degrees aperture
    octomap_exploration.depth_sensor = DepthSensor(225, 16, 3.141595, 30 * (M_PI/180.0), 4.0);

    octomap::point3d origin(0, 0, 0);
    octomap::point3d direction(0, 0, 0); // rad
    octomap::OcTree* octree = new octomap::OcTree(0.1f);
    octomap::Pointcloud hits = octomap_exploration.cast_rays(octree, origin, direction);

    visualization_msgs::MarkerArray marker_arr;

    while (ros::ok())
    {
        marker_arr.markers.clear();

        for(int i = 0; i < hits.size(); i++) {
            marker_arr.markers.push_back(
                    OctomapExploration::prepareMarkerROS(
                            hits.getPoint(i),
                            i,
                            visualization_msgs::Marker::CUBE,
                            "",
                            octomap::point3d(0.02, 0.02, 0.02),
                            1.0,
                            1.0,
                            1.0,
                            0.0,
                            "/os1_init",
                            15
                    ));
        }

        ROS_INFO_STREAM("point cloud size (hits): " << hits.size());

        marker_pub.publish(marker_arr);
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
//
// Created by h3ct0r on 27/09/20.
//

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

    ros::Publisher virtual_octomap_pub = nh.advertise<octomap_msgs::Octomap>("/virtual_octomap", 1);
    ros::Publisher frontier_mi_label_pub = nh.advertise<visualization_msgs::MarkerArray>(
                "/frontiers_mutual_information", 1, true);

    OctomapExploration octomap_exploration(nh);

    octomap::point3d origin(0, 0, 0);
    octomap::point3d direction(0, 0, 0);
    octomap::OcTree* octree = NULL;

    while (ros::ok())
    {
        octree = new octomap::OcTree(0.01f);

        double mi_before = octomap_exploration.get_free_volume(octree);

        octomap::Pointcloud hits = octomap_exploration.cast_rays(octree, origin, direction);

        octree->insertPointCloud(hits, origin, octomap_exploration.depth_sensor.max_range, true, true);

        double mi_after = octomap_exploration.get_free_volume(octree) - mi_before;

        ROS_DEBUG_STREAM("mi_before:" << mi_before << " mi_after:" << mi_after);

        octomap_msgs::Octomap virtual_octomap;
        bool res = octomap_msgs::binaryMapToMsg(*octree, virtual_octomap);
        virtual_octomap.header.frame_id = "/os1_sensor";
        virtual_octomap.header.stamp = ros::Time::now();

        ROS_DEBUG_STREAM("Convert to binaryMapMsg res:" << res);

        virtual_octomap_pub.publish(virtual_octomap);
        delete(octree);

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
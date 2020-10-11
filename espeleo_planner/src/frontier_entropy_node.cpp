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

#include "frontier_entropy.h"

using namespace std;
using namespace std::chrono;


int main(int argc, char **argv) {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::console::notifyLoggerLevelsChanged();

    ros::init(argc, argv, "octomap_frontier_entropy");
    ros::NodeHandle nh;
    OctomapExploration octomap_exploration(nh);

    ros::Duration(2.0).sleep();
    ros::Rate r(2);

    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}

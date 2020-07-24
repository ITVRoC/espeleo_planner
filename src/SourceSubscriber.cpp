//
// Created by fred on 14/05/2020.
//

#include "SourceSubscriber.h"

SourceSubscriber::SourceSubscriber() {}

SourceSubscriber::~SourceSubscriber() {}

void SourceSubscriber::sourceFromRVizCallback(const nav_msgs::OdometryConstPtr& msg) {
    odom = msg;
    poseSource = odom->pose.pose;
    // ROS_INFO("Selected Source points: [%f][%f][%f]", poseSource.position.x, poseSource.position.y,
    //         poseSource.position.z);
}

//
// Created by fred on 14/05/2020.
//

#ifndef PLANNING_INTEGRATED_SOURCESUBSCRIBER_H
#define PLANNING_INTEGRATED_SOURCESUBSCRIBER_H

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"

class SourceSubscriber {
public:
    nav_msgs::OdometryConstPtr odom;
    geometry_msgs::Pose poseSource;
    ros::NodeHandle n;
    ros::Subscriber sub;
    SourceSubscriber();
    ~SourceSubscriber();
    void sourceFromRVizCallback(const nav_msgs::OdometryConstPtr & msg);
};


#endif //PLANNING_INTEGRATED_SOURCESUBSCRIBER_H

//
// Created by fred on 14/05/2020.
//

#ifndef PLANNING_INTEGRATED_SOURCESUBSCRIBER_H
#define PLANNING_INTEGRATED_SOURCESUBSCRIBER_H

#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose.h"

class SourceSubscriber {
public:
    geometry_msgs::PoseWithCovarianceStampedConstPtr poseWCov;
    geometry_msgs::Pose poseSource;
    ros::NodeHandle n;
    ros::Subscriber sub;
    SourceSubscriber();
    ~SourceSubscriber();
    void sourceFromRVizCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg);
};


#endif //PLANNING_INTEGRATED_SOURCESUBSCRIBER_H

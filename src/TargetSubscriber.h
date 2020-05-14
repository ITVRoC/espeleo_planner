//
// Created by fred on 14/05/2020.
//

#ifndef PLANNING_INTEGRATED_TARGETSUBSCRIBER_H
#define PLANNING_INTEGRATED_TARGETSUBSCRIBER_H

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include <fstream>

class TargetSubscriber {
public:
    geometry_msgs::PoseStampedConstPtr poseStamped;
    ros::NodeHandle nodeH;
    ros::Subscriber sub;
    std::string topicName;
    TargetSubscriber();
    TargetSubscriber(std::string topicName);
    ~TargetSubscriber();
    void poseFromRVizCallback(const geometry_msgs::PoseStampedConstPtr& msg);

};


#endif //PLANNING_INTEGRATED_TARGETSUBSCRIBER_H

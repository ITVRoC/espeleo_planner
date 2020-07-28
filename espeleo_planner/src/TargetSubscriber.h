//
// Created by fred on 14/05/2020.
//

#ifndef PLANNING_INTEGRATED_TARGETSUBSCRIBER_H
#define PLANNING_INTEGRATED_TARGETSUBSCRIBER_H

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include <fstream>

class TargetSubscriber {
public:
    geometry_msgs::PointStampedConstPtr pointStamped;
    ros::NodeHandle nodeH;
    ros::Subscriber sub;
    std::string topicName;
    std::string pointCloudTopic;
    TargetSubscriber();
    TargetSubscriber(std::string topicName);
    ~TargetSubscriber();
    void poseFromRVizCallback(const geometry_msgs::PointStampedConstPtr& msg);
    void setPointCloudTopic(std::string pointCloudTopicName);

};


#endif //PLANNING_INTEGRATED_TARGETSUBSCRIBER_H

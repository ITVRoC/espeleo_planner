//
// Created by fred on 14/05/2020.
//

#include "TargetSubscriber.h"

#include <utility>
#include "ros/package.h"
#include "geometry_msgs/PointStamped.h"

TargetSubscriber::TargetSubscriber() {}

TargetSubscriber::TargetSubscriber(std::string topicName): topicName(std::move(topicName)) {}

TargetSubscriber::~TargetSubscriber() {}

void TargetSubscriber::poseFromRVizCallback(const geometry_msgs::PointStampedConstPtr &msg) {
    ROS_INFO("Selected Target points: [%f][%f][%f]", msg->point.x, msg->point.y, msg->point.z);
    pointStamped = msg;

    // Subscribing to the PointCloud2 topic only once
    sensor_msgs::PointCloud pc1Msg;
    sensor_msgs::PointCloud2ConstPtr pc2Msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>
            (pointCloudTopic);
    sensor_msgs::convertPointCloud2ToPointCloud((*pc2Msg), pc1Msg);

    // Put the Points from PointCloud in a CSV file
    std::ofstream outFile;
    std::string espeleo_planner_path = ros::package::getPath("espeleo_planner");
    std::string outFilePath = espeleo_planner_path + "/mapFiles/map.csv";
    outFile.open(outFilePath);
    outFile << "X,Y,Z" << std::endl;
    for(auto it: pc1Msg.points){
        std::string toFile = std::to_string(it.x) + "," + std::to_string(it.y) + "," + std::to_string(it.z);
        outFile << toFile << std::endl;
    }
    outFile.close();
}

void TargetSubscriber::setPointCloudTopic(std::string pointCloudTopicName) {
    pointCloudTopic = std::move(pointCloudTopicName);
}

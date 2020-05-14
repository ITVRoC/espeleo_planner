//
// Created by fred on 14/05/2020.
//

#include "TargetSubscriber.h"

TargetSubscriber::TargetSubscriber() {}

TargetSubscriber::TargetSubscriber(std::string topicName): topicName(topicName) {}

TargetSubscriber::~TargetSubscriber() {}

void TargetSubscriber::poseFromRVizCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
    ROS_INFO("Selected points: [%f][%f][%f]", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    poseStamped = msg;

    // Subscribing to the PointCloud2 topic only once
    sensor_msgs::PointCloud pc1Msg;
    sensor_msgs::PointCloud2ConstPtr pc2Msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>
            ("/kinect/depth_registered/points");
    sensor_msgs::convertPointCloud2ToPointCloud((*pc2Msg), pc1Msg);

    // Put the Points from PointCloud in a CSV file
    std::ofstream outFile;
    std::string outFilePath = "/home/fred/catkin_ws/src/planning_integrated/mapFiles/map.csv";
    outFile.open(outFilePath);
    outFile << "X,Y,Z" << std::endl;
    for(auto it: pc1Msg.points){
        std::string toFile = std::to_string(it.x) + "," + std::to_string(it.y) + "," + std::to_string(it.z);
        outFile << toFile << std::endl;
    }
    outFile.close();
}
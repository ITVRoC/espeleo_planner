//
// Created by fred on 11/05/2020.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include <fstream>
#include <vector>
#include <iostream>
#include <sstream>
#include <string>
#include "sensor_msgs/point_cloud_conversion.h"


void poseFromRVizCallback(const geometry_msgs::PoseStampedConstPtr& msg){
    ROS_INFO("Selected points: [%f][%f][%f]", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

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


std::vector<std::string> splitOneChar(const std::string &s, char delim) {
    std::vector<std::string> result;
    std::stringstream ss (s);
    std::string item;

    while (getline (ss, item, delim)) {
        result.push_back (item);
    }

    return result;
}

std::vector<std::string> split (std::string s, std::string delimiter) {
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    std::string token;
    std::vector<std::string> res;

    while ((pos_end = s.find (delimiter, pos_start)) != std::string::npos) {
        token = s.substr (pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back (token);
    }

    res.push_back (s.substr (pos_start));
    return res;
}

void readFile(const std::string& filename, std::vector<std::vector<double>>* pointsVector){
    std::ifstream filePointer;
    std::vector<double> points;
    std::vector<std::string> strPoints;
    std::string in;
    std::string::size_type sizeType;
    filePointer.open(filename);
    if (!filePointer){
        ROS_ERROR("File not loaded.");
    }
    std::string delimiter = ",";
    while(filePointer >> in){
        strPoints = split(in, delimiter);
        std::cout << strPoints[0] << strPoints[1] << strPoints[2] << std::endl;
        points.push_back(std::stod(strPoints[0],&sizeType));
        points.push_back(std::stod(strPoints[1],&sizeType));
        points.push_back(std::stod(strPoints[2],&sizeType));
        pointsVector->push_back(points);
        points.clear();
    }
}

void publishPathOnRViz(ros::Publisher* pathPub,std::vector<std::vector<double>>* pointsVector){
    nav_msgs::Path pathToPublish;
    for(auto it: *pointsVector){
        geometry_msgs::PoseStamped poseSt;
        poseSt.pose.position.x = it[0];
        poseSt.pose.position.y = it[1];
        poseSt.pose.position.z = it[3];
        pathToPublish.poses.push_back(poseSt);
    }
    ROS_INFO("Publishing path...");
    //int count = 0;
    //while(ros::ok()){
    pathPub->publish(pathToPublish);
    //    ros::spinOnce();
    //    ++count;
    //}
}


int main(int argc, char **argv) {
    ros::Time::init();
    ROS_INFO("Starting node");

    ros::init(argc, argv, "integrated_planner");
    ros::NodeHandle nodeHandler;
    std::vector<std::vector<double>> pointsVector;

    //ros::Subscriber on RViz clicked 2D Nav Point
    ros::Subscriber sub = nodeHandler.subscribe("/move_base_simple/goal", 1000, poseFromRVizCallback);
    //readFile("/home/fred/catkin_ws/src/planning_integrated/test_files/plotPoints.txt", &pointsVector);


    //Surface Reconstruction Paths
    std::string outFilePath = "/home/fred/catkin_ws/src/planning_integrated/mapFiles/map.csv";
    std::string stlOutput = "/home/fred/catkin_ws/src/planning_integrated/include/";
    std::ifstream fileCheck;
    //File pointer to check whether there is a csv file or not, if topic /move_base_simple/goal received any message

    while(ros::ok()) {
        fileCheck.open(outFilePath);
        if (!fileCheck) {
            ROS_INFO("RViz did not receive start point yet.");
        } else {
            //System Call to Surface Reconstruction Algorithm and STL Generation
            system("cd /home/fred/catkin_ws/src/planning_integrated/include/surface_recon/build/ &&"
                   " ./recon_surface --csv /home/fred/catkin_ws/src/planning_integrated/mapFiles/map.csv "
                   "--output /home/fred/catkin_ws/src/planning_integrated/mapFiles/ --holemaxsize 150");

            // Delete .csv file to run this code only when RViz 2D Nav goal is clicked
            std::string command = "rm " + outFilePath;
            int n = command.length();
            char command_f[n + 1];
            strcpy(command_f, command.c_str());
            system(command_f);

            //System Call to Python Planning Algorithm


            //Publish Path On RViz on Main

        }
        ros::Duration(4).sleep();
        ROS_INFO("LOOP");
        ros::spinOnce();
    }


    //Path Publisher on RViz
    //ros::Publisher pathPublisher = nodeHandler.advertise<nav_msgs::Path>("/robot_path", 1000);
    //ros::Rate loop_rate(10);

    // ros::Subscriber subs = n.subscribe("/kinect/depth_registered/points", 1000, fromPointCloudCallback);

    //ros::Subscirber to robot origin point


    //publishPathOnRViz(&pathPublisher, &pointsVector);
    //ros::spin();
    return 0;
}
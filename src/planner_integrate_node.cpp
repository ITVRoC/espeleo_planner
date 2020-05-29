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
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "TargetSubscriber.h"
#include "SourceSubscriber.h"


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
    pointsVector->clear();
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
        //std::cout << strPoints[0] << strPoints[1] << strPoints[2] << std::endl;
        points.push_back(std::stod(strPoints[0],&sizeType));
        points.push_back(std::stod(strPoints[1],&sizeType));
        points.push_back(std::stod(strPoints[2],&sizeType));
        pointsVector->push_back(points);
        points.clear();
    }
}

void publishPathOnRViz(ros::Publisher* pathPub,std::vector<std::vector<double>>* pointsVector){
    nav_msgs::Path pathToPublish;
    pathToPublish.header.frame_id = "map";
    //pathToPublish.header.frame_id = "kinect_link";
    //pathToPublish.header.frame_id = "test";
    for(auto it: *pointsVector){
        geometry_msgs::PoseStamped poseSt;
        poseSt.pose.position.x = it[0];
        poseSt.pose.position.y = it[1];
        poseSt.pose.position.z = it[3];
        pathToPublish.poses.push_back(poseSt);
    }
    ROS_INFO("Publishing path...");
    pathPub->publish(pathToPublish);
}


int main(int argc, char **argv) {
    ros::Time::init();
    ROS_INFO("Node Started.");

    ros::init(argc, argv, "integrated_planner");
    ros::NodeHandle nodeHandler;
    std::vector<std::vector<double>> pointsVector;

    std::vector<std::string> metrics;
    metrics.push_back("Shortest");
    metrics.push_back("Energy");
    metrics.push_back("Transverse");
    metrics.push_back("Combined");


    //ros::Subscriber on RViz clicked 2D Nav Point
    //ros::Subscriber sub = nodeHandler.subscribe("/move_base_simple/goal", 1000, poseFromRVizCallback);
    TargetSubscriber targetPos;
    ros::Subscriber sub = nodeHandler.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000,
            &TargetSubscriber::poseFromRVizCallback, &targetPos);
    //targetPos.setPointCloudTopic("/octomap_point_cloud_centers");
    //targetPos.setPointCloudTopic("/kinect/depth_registered/points");
    targetPos.setPointCloudTopic("/rtabmap/cloud_map");
    //readFile("/home/fred/catkin_ws/src/planning_integrated/test_files/plotPoints.txt", &pointsVector);

    SourceSubscriber sourcePos;
    ros::Subscriber sub_source = nodeHandler.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1000,
            &SourceSubscriber::sourceFromRVizCallback, &sourcePos);


    //Surface Reconstruction Paths
    std::string outFilePath = "/home/fred/catkin_ws/src/planning_integrated/mapFiles/map.csv";
    //std::string stlOutput = "/home/fred/catkin_ws/src/planning_integrated/include/surface_recon/build/mesh.stl";
    std::string stlOutput = "/home/fred/catkin_ws/src/planning_integrated/mapFiles/meshObj.obj";
    std::ifstream fileCheck;
    //File pointer to check whether there is a csv file or not, if topic /move_base_simple/goal received any message

    //Path Publisher on RViz
    ros::Publisher pathPublisher_short = nodeHandler.advertise<nav_msgs::Path>("/robot_path_shortest", 1000);
    ros::Publisher pathPublisher_energ = nodeHandler.advertise<nav_msgs::Path>("/robot_path_energy", 1000);
    ros::Publisher pathPublisher_transv = nodeHandler.advertise<nav_msgs::Path>("/robot_path_transversal", 1000);
    ros::Publisher pathPublisher_comb = nodeHandler.advertise<nav_msgs::Path>("/robot_path_combined", 1000);
    std::vector<ros::Publisher*> pathPublishers;
    pathPublishers.push_back(&pathPublisher_short);
    pathPublishers.push_back(&pathPublisher_energ);
    pathPublishers.push_back(&pathPublisher_transv);
    pathPublishers.push_back(&pathPublisher_comb);
    ros::Rate loop_rate(10);

    while(ros::ok()) {
        fileCheck.open(outFilePath);
        if (!fileCheck) {
            ROS_INFO("RViz did not receive target point yet.");
        } else {
            //System Call to Surface Reconstruction Algorithm and STL Generation
            system("cd /home/fred/catkin_ws/src/planning_integrated/include/surface_recon/build/ &&"
                   " ./recon_surface --csv /home/fred/catkin_ws/src/planning_integrated/mapFiles/map.csv "
                   "--output /home/fred/catkin_ws/src/planning_integrated/mapFiles/meshObj --holemaxsize 30"); // 150


            // Delete .csv file to run this code only when RViz 2D Nav goal is clicked
            std::string command = "rm " + outFilePath;
            int n = command.length();
            char command_f[n + 1];
            strcpy(command_f, command.c_str());
            system(command_f);

            //String for python System Call
            std::string pythonCommand = "python3 /home/fred/catkin_ws/src/planning_integrated/scripts/plann_call.py " +
                    stlOutput + " " + std::to_string(sourcePos.poseSource.position.x) + " " +
                    std::to_string(sourcePos.poseSource.position.y) + " " + std::to_string(targetPos.poseStamped->pose.position.x)
                    + " " + std::to_string(targetPos.poseStamped->pose.position.y);

            //System Call to Python Planning Algorithm
            n = pythonCommand.length();
            char pythonC[n+1];
            strcpy(pythonC, pythonCommand.c_str());
            system(pythonC);

            //Publish Path On RViz on Main
            int index = 0;
            for(const auto& it: metrics) {
                std::string comm = "/home/fred/catkin_ws/devel/lib/planning_integrated/TxtPaths/DijkstraPoints"
                        + it + ".txt";
                std::cout << comm << std::endl;
                readFile(comm, &pointsVector);
                publishPathOnRViz(pathPublishers.at(index), &pointsVector);
                index++;
            }

        }
        ros::Duration(4).sleep();
        ros::spinOnce();
    }

    return 0;
}
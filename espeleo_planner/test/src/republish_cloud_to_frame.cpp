//
// Created by h3ct0r on 28/09/20.
//

#include <iostream>
#include <fstream>
#include <chrono>
#include <algorithm>
#include <iterator>

#include <octomap/octomap.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>
#include <tf/transform_broadcaster.h>
#include <ros/console.h>
#include <octomap/math/Utils.h>
#include <octomap/math/Vector3.h>
#include <map>
#include <limits>
#include "boost/tuple/tuple_comparison.hpp"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/distances.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/range/algorithm.hpp>

#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <math.h>
#include <limits>

#define degToRad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)
#define DEBUG  1

using namespace std;
using namespace std::chrono;

ros::Publisher cloud_transformed_world;
sensor_msgs::PointCloud2 cloud_msg;
auto start = chrono::high_resolution_clock::now();
auto stop = chrono::high_resolution_clock::now();


tf::StampedTransform lookup_transform(const std::string& target_frame, const std::string& source_frame) {
    tf::StampedTransform transform;

    tf::TransformListener tf_listener;
    try {
        tf_listener.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(10.0) );
        tf_listener.lookupTransform (target_frame, source_frame, ros::Time(0), transform);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
    }

    return transform;
}

void transformPointCloud(const std::string &target_frame, const sensor_msgs::PointCloud2 &in,
                         pcl::PointCloud<pcl::PointXYZ> &out) {
    sensor_msgs::PointCloud2::Ptr transformed_cloud(new sensor_msgs::PointCloud2);
    pcl_ros::transformPointCloud(target_frame, lookup_transform(target_frame, in.header.frame_id), in, *transformed_cloud);

    pcl::fromROSMsg(*transformed_cloud, out);
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input){
    if(cloud_msg.height <= 0){
        ROS_INFO_STREAM("cloud received...");
    }

    start = chrono::high_resolution_clock::now();

    string target_frame = "/world";
    sensor_msgs::PointCloud2 cloud = *input;

    sensor_msgs::PointCloud2::Ptr transformed_cloud(new sensor_msgs::PointCloud2);
    pcl_ros::transformPointCloud(target_frame, lookup_transform(target_frame, cloud.header.frame_id), cloud, *transformed_cloud);

    stop = chrono::high_resolution_clock::now();
    ROS_INFO_STREAM("Time to process cloud: " << float(chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0) << " seconds");

    cloud_transformed_world.publish(transformed_cloud);
}

//void process_cloud(){
//    if(cloud_msg.height <= 0){
//        ROS_WARN_ONCE("Cloud not received yet, waiting for new pointcloud msg...");
//        return;
//    }
//
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_original(new pcl::PointCloud<pcl::PointXYZ>);
//
//    pcl::PCLPointCloud2 cloud_tmp;
//    pcl_conversions::toPCL(cloud_msg, cloud_tmp); // sensor_msgs::PointCloud2 to pcl::PCLPointCloud2
//    pcl::fromPCLPointCloud2(cloud_tmp, *cloud_original); // pcl::PCLPointCloud2 to PointCloud
//
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    *cloud = *cloud_original;
//
//    try{
//            cam_bl.waitForTransform("base_link","pico_flexx_optical_frame",ros::Time(0),ros::Duration(5));
//            cam_bl.lookupTransform("base_link","pico_flexx_optical_frame",ros::Time(0),cam_bl_tf);
//    }
//    catch(tf::TransformException &ex){
//            ROS_WARN("%s",ex.what());
//    };
//
//    sensor_msgs::PointCloud2 input_in_bl;
//    pcl_ros::transformPointCloud("base_link",*input_from_camera_frame,input_in_bl,cam_bl);
//}


int main(int argc, char **argv) {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::console::notifyLoggerLevelsChanged();

    ros::init(argc, argv, "republish_cloud_to_frame");
    ros::NodeHandle nh;
    ros::Rate r(10);

    cloud_transformed_world = nh.advertise<sensor_msgs::PointCloud2>("/cloud_transformed_world", 1);
    ros::Subscriber sub = nh.subscribe("/laser_cloud_surround2", 1, cloud_cb);

    ros::Duration(2.0).sleep();
    ros::spin();

    return 0;
}
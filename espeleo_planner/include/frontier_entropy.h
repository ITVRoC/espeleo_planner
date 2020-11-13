//
// Created by h3ct0r on 27/09/20.
//

#ifndef ESPELEO_PLANNER_FRONTIER_ENTROPY_H
#define ESPELEO_PLANNER_FRONTIER_ENTROPY_H

#endif //ESPELEO_PLANNER_FRONTIER_ENTROPY_H

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
#include "espeleo_planner/processFrontier.h"
#include "espeleo_planner/processAllFrontiers.h"
#include "espeleo_planner/processAllFrontiersUsingSTLOctomap.h"
#include <mutex>

#include "depth_sensor.h"
#include "frontier_information.h"

using namespace std;
using namespace std::chrono;

class PointCloudPub {
    public:
        PointCloudPub(ros::NodeHandle _nh) : nh(_nh), msg(new pcl::PointCloud<pcl::PointXYZ>), tp_name("virtual_scans") {
            msg->header.frame_id = pointcloud_frame_id;
            msg->height = 1;
            msg->width = 0;
            pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(tp_name, 1);
        }

        void insert_point3d(double x, double y, double z) {
            msg->points.push_back(pcl::PointXYZ(x, y, z));
            msg->width++;
        }

        void clear() {
            msg->points.clear();
            msg->width = 0;
        }

        void publish() const {
            msg->header.stamp = ros::Time::now().toNSec() / 1e3;
            pub.publish(msg);
            ros::spinOnce();
        }

    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        const string pointcloud_frame_id = "/os1_init";
        pcl::PointCloud<pcl::PointXYZ>::Ptr msg;
        string tp_name;
};

class OctomapExploration {
    private:
        ros::NodeHandle nh;
        const string OCTOMAP_BINARY_TOPIC = "/octomap_binary";
        const string FRONTIER_CENTERS_MARKERS_TOPIC = "/frontiers_ground_centroids";
        const string PROCESS_FRONTIER_BY_ID_SRV_NAME = "/process_frontier_by_id";
        const string PROCESS_ALL_FRONTIERS_SRV_NAME = "/process_all_frontiers";
        const string PROCESS_ALL_FRONTIERS_STL_SRV_NAME = "/process_all_frontiers_stl";
        const string BASE_FRAME_ID = "/os1_init";

        ros::Subscriber octomap_sub;
        ros::Subscriber frontier_centers_sub;

        PointCloudPub pointcloud_pub;
        PointCloudPub CurrentPcl_pub;

        ros::Publisher virtual_octomap_pub = nh.advertise<octomap_msgs::Octomap>("/virtual_octomap", 1, true);
        ros::Publisher frontier_mi_label_pub = nh.advertise<visualization_msgs::MarkerArray>(
                "/frontiers_mutual_information", 1, true);

        ros::ServiceServer process_frontier_srv;
        ros::ServiceServer process_all_frontiers_srv;
        ros::ServiceServer process_all_frontiers_stl_srv;

        octomap::OcTree *octomap_load = NULL;
        octomap::OcTree *octomap_from_stl = NULL;

        octomap_msgs::Octomap::ConstPtr octomap_binary;
        visualization_msgs::MarkerArray::ConstPtr frontier_centers_array;
        octomap::point3d position;
        octomap::point3d orientation;
        std::map<boost::tuple<double, double, double>, FrontierInformation> frontierInfoMap;

        mutex octomap_mutex;    // a mutex to resolve the race conditions.
    public:

        /* Velodyne VPL16 FOV 30 degrees vertical (-+ 15), and 360 degrees horizontal
           16 vertical lines
           3600 points horizontal (simplified to 450 to ease calculations) */
        DepthSensor depth_sensor = DepthSensor(900, 16, 6.28319, 20 * (M_PI/180.0), 15.0);

        OctomapExploration(ros::NodeHandle _nh) : nh(_nh), pointcloud_pub(_nh), CurrentPcl_pub(_nh) {
            position = octomap::point3d(0, 0, 0);
            orientation = octomap::point3d(0, 0, 0);

            octomap_sub = nh.subscribe<octomap_msgs::Octomap>(OCTOMAP_BINARY_TOPIC, 1,
                    &OctomapExploration::octomap_callback, this);

            frontier_centers_sub = nh.subscribe<visualization_msgs::MarkerArray>(FRONTIER_CENTERS_MARKERS_TOPIC, 1,
                    &OctomapExploration::frontier_centers_callback, this);

            process_frontier_srv = nh.advertiseService(PROCESS_FRONTIER_BY_ID_SRV_NAME,
                    &OctomapExploration::processFrontierByIdSrvCallback, this);

            process_all_frontiers_srv = nh.advertiseService(PROCESS_ALL_FRONTIERS_SRV_NAME,
                    &OctomapExploration::processAllFrontiersSrvCallback, this);

            process_all_frontiers_stl_srv = nh.advertiseService(PROCESS_ALL_FRONTIERS_STL_SRV_NAME,
                    &OctomapExploration::processAllFrontiersUsingSTLOctomapSrvCallback, this);
        }

        ~OctomapExploration() {
        }

        void octomap_callback(const octomap_msgs::Octomap::ConstPtr &msg);

        void frontier_centers_callback(const visualization_msgs::MarkerArray::ConstPtr &msg);

        octomap::Pointcloud cast_rays(const octomap::OcTree *octree, const octomap::point3d &position,
                const octomap::point3d &orientation) const;

        double calc_MI(octomap::OcTree *octree, const octomap::point3d &origin, const octomap::Pointcloud &hits,
                const double before) const;

        void publish_virtual_octomap(octomap::OcTree *octree) const;
        void publish_frontier_text_labels(std::vector<double> frontier_mi, std::vector<octomap::point3d> frontier_pos);

        std::vector<double> process_octomap(octomap::OcTree *octomap_base_param);

        bool processFrontierByIdSrvCallback(espeleo_planner::processFrontier::Request &req,
                                            espeleo_planner::processFrontier::Response &res);

        bool processAllFrontiersSrvCallback(espeleo_planner::processAllFrontiers::Request &req,
                                            espeleo_planner::processAllFrontiers::Response &res);

        bool processAllFrontiersUsingSTLOctomapSrvCallback(espeleo_planner::processAllFrontiersUsingSTLOctomap::Request &req,
                                            espeleo_planner::processAllFrontiersUsingSTLOctomap::Response &res);


        octomap::point3d convert3DTupleToPoint3D(boost::tuple<double, double, double> p1_boost) {
            return octomap::point3d(
                    p1_boost.get<0>(),
                    p1_boost.get<1>(),
                    p1_boost.get<2>()
            );
        }

        boost::tuple<double, double, double> convertPoint3Dto3DTuple(octomap::point3d p1) {
            return boost::make_tuple((double) p1.x(), (double) p1.y(), (double) p1.z());
        }

        double get_free_volume(const octomap::OcTree *octree) const {
            double volume = 0;
            for (octomap::OcTree::leaf_iterator n = octree->begin_leafs(octree->getTreeDepth());
                 n != octree->end_leafs(); ++n) {
                if (!octree->isNodeOccupied(*n))
                    volume += pow(n.getSize(), 3);
                // cout << "node : " << n.getCoordinate() << endl;
            }
            return volume;
        }

        static visualization_msgs::Marker prepareMarkerROS(octomap::point3d origin, int id, uint32_t shape,
                std::string label, octomap::point3d scale, float a, float r, float g, float b, string frame_id,
                int duration) {

            visualization_msgs::Marker marker;
            marker.header.frame_id = frame_id;
            marker.header.stamp = ros::Time::now();
            marker.id = id;

            marker.type = shape;

            marker.text = label;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = origin.x();
            marker.pose.position.y = origin.y();
            marker.pose.position.z = origin.z();

            marker.scale.x = scale.x();
            marker.scale.y = scale.y();
            marker.scale.z = scale.z();

            marker.color.a = a;
            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;

            marker.lifetime = ros::Duration(duration);

            return marker;
        }
};
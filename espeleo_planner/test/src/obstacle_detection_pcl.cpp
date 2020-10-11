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

#include <math.h>
#include <limits>

#define degToRad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)
#define DEBUG  1

using namespace std;
using namespace std::chrono;

ros::Publisher pub_obstacles_markers;
ros::Publisher pub_closest_obstacle_marker;
ros::Publisher pub_closest_obstacle_pt;

sensor_msgs::PointCloud2 cloud_msg;

//pcl::visualization::PCLVisualizer viewer("cloud_viewer");
//#ifndef DEBUG
//    viewer.close();
//#endif

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input){
    if(cloud_msg.height <= 0){
        ROS_INFO_STREAM("cloud received...");
    }

    cloud_msg = *input;
}

visualization_msgs::Marker prepare_marker(double x, double y, double z, double scale, int id, double r, double g,
        double b, string frame_id){
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();

    marker.id = id;
    marker.type = 2;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration(2);

    return marker;
}

double fov_segmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double min_fov, double max_fov, pcl::PointCloud<pcl::PointXYZ>::Ptr keypoint_cloud){
    boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::sum , boost::accumulators::tag::variance, boost::accumulators::tag::mean > > acc;

    for (size_t i = 0; i < (*cloud).points.size(); ++i) {
        double x = (*cloud).points[i].x;
        double y = (*cloud).points[i].y;
        double angle = atan2(y, x);

        if(angle >= degToRad(-max_fov) && angle <= degToRad(-min_fov)){
            (*keypoint_cloud).push_back((*cloud).points[i]);
            acc((*cloud).points[i].z);
        }
    }

    return sqrt(boost::accumulators::variance(acc));
}

void process_cloud(){
    if(cloud_msg.height <= 0){
        ROS_WARN_ONCE("Cloud not received yet, waiting for new pointcloud msg...");
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_original(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCLPointCloud2 cloud_tmp;
    pcl_conversions::toPCL(cloud_msg, cloud_tmp); // sensor_msgs::PointCloud2 to pcl::PCLPointCloud2
    pcl::fromPCLPointCloud2(cloud_tmp, *cloud_original); // pcl::PCLPointCloud2 to PointCloud

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud = *cloud_original;

    ROS_INFO_STREAM("cloud_original size:" << cloud_original->size());
    ROS_INFO_STREAM("cloud size:" << cloud->size());

    double zMin, zMax, xMin, xMax, yMin, yMax = 2.5;
    pcl::PassThrough<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);

    filter.setFilterFieldName("z");
    filter.setFilterLimits(-3.0, 3.0);
    filter.setNegative(false);
    filter.filter(*cloud);

    filter.setFilterFieldName("z");
    filter.setFilterLimits(-0.38, -0.3);
    filter.setNegative(true);
    filter.filter(*cloud);

    filter.setFilterFieldName("y");
    filter.setFilterLimits(-3.0, 3.0);
    filter.setNegative(false);
    filter.filter(*cloud);

    filter.setFilterFieldName("x");
    filter.setFilterLimits(-3.0, 3.0);
    filter.setNegative(false);
    filter.filter(*cloud);

    ROS_INFO_STREAM("cloud size:" << cloud->size());

    // Perform the actual filtering
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.01, 0.01, 0.01);
    sor.filter(*cloud_filtered);

    ROS_INFO_STREAM("cloud_filtered size:" << cloud_filtered->size());

    // binning by angle and filter by std deviation on Z values
    double n_bins = 72;
    double bin_size = 360/n_bins;
    vector<int> fov_sign{-1, 1};
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_binned (new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i = 0; i < (n_bins / 2); ++i) {
        for (size_t j = 0; j < fov_sign.size(); ++j) {
            double sign = fov_sign[j];
            double bin_start = (i * bin_size) * sign;
            double bin_end = ((i + 1) * bin_size) * sign;

            if(sign < 0){
                double temp = bin_start;
                bin_start = bin_end;
                bin_end = temp;
            }

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_fov (new pcl::PointCloud<pcl::PointXYZ>);
            double std_dev = fov_segmentation(cloud_filtered, bin_start, bin_end, cloud_fov);

            if(std_dev > 0.1){
                *cloud_binned += *cloud_fov;
            }
        }
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clustered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_binned);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.1);
    ec.setMinClusterSize(200);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_binned);
    ec.extract(cluster_indices);

    visualization_msgs::MarkerArray marker_array;
    pcl::PointXYZ origin = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ closest_point;
    double closest_point_dist = std::numeric_limits<double>::infinity();
    int c_idx = -1;

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        c_idx++;
        boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::sum, boost::accumulators::tag::variance, boost::accumulators::tag::mean>> acc;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
            cluster->push_back((*cloud_binned).points[*pit]);
            acc((*cloud_binned).points[*pit].z);
        }

        double std_dev = sqrt(boost::accumulators::variance(acc));

        if(std_dev > 0.1){
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter_outlier (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(cluster);
            sor.setMeanK(50);
            sor.setStddevMulThresh(1.0);
            sor.filter(*cloud_filter_outlier);

            /*pcl::PointXYZ centroid;
            pcl::computeCentroid(*cloud_filter_outlier, centroid);

            //double x, double y, double z, double scale, int id, double r, double, g, double b
            visualization_msgs::Marker marker = prepare_marker(centroid.x, centroid.y, centroid.z, 0.1, c_idx, 0.9, 0.1, 0.1);
            marker_array.markers.push_back(marker);*/

            pcl::KdTree<pcl::PointXYZ>::Ptr cluster_tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
            cluster_tree->setInputCloud(cloud_filter_outlier);

            std::vector<int> nn_indices (1);
            std::vector<float> nn_dists (1);

            cluster_tree->nearestKSearch(origin, 1, nn_indices, nn_dists);

            visualization_msgs::Marker marker = prepare_marker(
                    cloud_filter_outlier->points[nn_indices[0]].x,
                    cloud_filter_outlier->points[nn_indices[0]].y,
                    cloud_filter_outlier->points[nn_indices[0]].z,
                    0.1,
                    c_idx,
                    0.9, 0.1, 0.1,
                    cloud_msg.header.frame_id);

            marker_array.markers.push_back(marker);

            *cloud_clustered += *cloud_filter_outlier;

            double d = pcl::squaredEuclideanDistance(origin, cloud_filter_outlier->points[nn_indices[0]]);
            if(d < closest_point_dist){
                closest_point_dist = d;
                closest_point = cloud_filter_outlier->points[nn_indices[0]];
            }

        }

        ROS_INFO_STREAM("cluster size:" << cluster->size() << " std_dev:" << std_dev);
    }

    ROS_INFO_STREAM("closer point dist:" << closest_point_dist);

    if(closest_point_dist < std::numeric_limits<double>::infinity()){
        visualization_msgs::Marker closest_marker = prepare_marker(
                closest_point.x,
                closest_point.y,
                closest_point.z,
                0.25,
                0,
                1.0, 0.0, 0.0,
                cloud_msg.header.frame_id);

        geometry_msgs::Point closest_pt_msg = geometry_msgs::Point();
        closest_pt_msg.x =  closest_point.x;
        closest_pt_msg.y =  closest_point.y;
        closest_pt_msg.z =  closest_point.z;

        pub_closest_obstacle_pt.publish(closest_pt_msg);
        pub_closest_obstacle_marker.publish(closest_marker);
        pub_obstacles_markers.publish(marker_array);
    }

//    #ifdef DEBUG
//        viewer.addPointCloud<pcl::PointXYZ>(cloud_clustered, "", 0);
//        viewer.spinOnce();
//        viewer.removeAllPointClouds();
//        viewer.removeAllShapes();
//    #endif

    // reset the pointcloud
    cloud_msg = sensor_msgs::PointCloud2();
}

int main(int argc, char **argv) {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::console::notifyLoggerLevelsChanged();

    ros::init(argc, argv, "obstacle_detection_pcd");
    ros::NodeHandle nh;
    ros::Rate r(10);

    pub_obstacles_markers = nh.advertise<visualization_msgs::MarkerArray>("/obstacles_points_markers", 1);
    pub_closest_obstacle_marker = nh.advertise<visualization_msgs::Marker>("/closest_obstacle_marker", 1);
    pub_closest_obstacle_pt = nh.advertise<geometry_msgs::Point>("/closest_obstacle_point", 1);

    ros::Subscriber sub = nh.subscribe("/os1_cloud_node/points", 1, cloud_cb);

    auto start = chrono::high_resolution_clock::now();
    auto stop = chrono::high_resolution_clock::now();

    ros::Duration(3.0).sleep();

    while (ros::ok())
    {
        start = chrono::high_resolution_clock::now();

        process_cloud();

        stop = chrono::high_resolution_clock::now();
        ROS_INFO_STREAM("Time to process cloud: " << float(chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0) << " seconds");

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
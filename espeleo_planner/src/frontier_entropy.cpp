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
#include <cstdlib>
#include <boost/filesystem/convenience.hpp>

#include "frontier_entropy.h"

using namespace std;
using namespace std::chrono;


void OctomapExploration::octomap_callback(const octomap_msgs::Octomap::ConstPtr &msg) {
    ROS_INFO_ONCE("octomap_callback called");

    std::unique_lock<std::mutex> lock(octomap_mutex);
    if (octomap_load != NULL) {
        //ROS_DEBUG_STREAM("octomap_load is not NULL, cleaning octomap");
        delete octomap_load;
    }

    octomap_load = dynamic_cast<octomap::OcTree *>(octomap_msgs::msgToMap(*msg));
}

void OctomapExploration::frontier_centers_callback(const visualization_msgs::MarkerArray::ConstPtr &msg) {
    ROS_INFO_ONCE("frontier_marker_callback");
    frontier_centers_array = msg;

    int frontier_size = frontier_centers_array->markers.size();
    //ROS_INFO("Frontier size: %d", frontier_size);
    if (frontier_size <= 0) {
        //ROS_ERROR("frontier_centers_array is empty, not updating frontierInfoMap map");
        return;
    }

//    std::chrono::milliseconds current_ms = duration_cast<milliseconds>(
//            system_clock::now().time_since_epoch()
//    );
//
//    // iterate all the frontiers received
//    for (int i = 0; i < frontier_size; ++i) {
//        double x1 = frontier_centers_array->markers[i].pose.position.x;
//        double y1 = frontier_centers_array->markers[i].pose.position.y;
//        double z1 = frontier_centers_array->markers[i].pose.position.z;
//        z1 += 0.5; // simulate Xtion position that is half meter over the groudn frontier center TODO: use the real TF for this!
//        boost::tuple<double, double, double> p1_boost = boost::make_tuple(x1, y1, z1);
//
//        // if this frontier is already in the map
//        // then only update the last updated time
//        auto it = frontierInfoMap.find(p1_boost);
//        if (it != frontierInfoMap.end()) {
//            it->second.last_updated_ms = current_ms;
//            continue;
//        }
//
//        // if is not in the map then add it and update the last updated time
//        auto fInfo = FrontierInformation(p1_boost);
//        fInfo.last_updated_ms = current_ms;
//        frontierInfoMap.insert(std::pair<boost::tuple<double, double, double>, FrontierInformation>(p1_boost, fInfo));
//    }
//
//    // remove old frontiers by last_updated_ms
//    for (auto it = frontierInfoMap.cbegin(); it != frontierInfoMap.cend();) {
//        if (it->second.last_updated_ms != current_ms) {
//            ROS_DEBUG_STREAM("[DEBUG] removing frontier:" << convert3DTupleToPoint3D(it->first) << "\t"
//                                                          << it->second.entropy);
//            frontierInfoMap.erase(it++);    // or "it = m.erase(it)" since C++11
//        } else {
//            ++it;
//        }
//    }
//
//    // debug
//    for (auto it = frontierInfoMap.begin(); it != frontierInfoMap.end(); ++it) {
//        ROS_DEBUG_STREAM(
//                "[DEBUG] frontierInfoMap:" << convert3DTupleToPoint3D(it->first) << "\t" << it->second.entropy);
//    }
}

octomap::Pointcloud OctomapExploration::cast_rays(const octomap::OcTree *octree, const octomap::point3d &position,
                                          const octomap::point3d &orientation) const {
    high_resolution_clock::time_point t1 = high_resolution_clock::now();

    octomap::Pointcloud hits;

    octomap::Pointcloud rays_to_cast;
    rays_to_cast.push_back(depth_sensor.sensor_rays);
    rays_to_cast.rotate(orientation.x(), orientation.y(), orientation.z());
    octomap::point3d end;

    // Cast Rays to 3d OctoTree and get hit points
    for(int i = 0; i < rays_to_cast.size(); i++) {
        if(octree->castRay(position, rays_to_cast.getPoint(i), end, true, depth_sensor.max_range)) {
            hits.push_back(end);
        } else {
            end = rays_to_cast.getPoint(i) * depth_sensor.max_range;
            end += position;
            hits.push_back(end);
        }
    }

    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
    ROS_DEBUG_STREAM("Time on ray cast: " << duration << "ms");
    //ROS_DEBUG_STREAM("\thits: " << hits.size());

    return hits;
}

double OctomapExploration::calc_MI(octomap::OcTree *octree, const octomap::point3d &origin, const octomap::Pointcloud &hits,
               const double before) const {
    high_resolution_clock::time_point t1 = high_resolution_clock::now();

    auto octree_copy = new octomap::OcTree(*octree);
    // TODO REMOVE THIS
    //publish_virtual_octomap(octree_copy);

    octree_copy->insertPointCloud(hits, origin, depth_sensor.max_range, false, false);
    double after = get_free_volume(octree_copy) + 0.01;

    publish_virtual_octomap(octree_copy);
    ros::Duration(5.0).sleep();
    // TODO TEMPORAL --- THIS IS TO ONLY PUBLISH THE FILLED OCTOMAP

    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
    ROS_DEBUG_STREAM("Time estimating MI: " << duration << "ms");

    delete octree_copy;

    return after - before;
}

void OctomapExploration::publish_virtual_octomap(octomap::OcTree *octree) const{

    octomap_msgs::Octomap virtual_octomap;
    bool res = octomap_msgs::binaryMapToMsg(*octree, virtual_octomap);
    virtual_octomap.header.frame_id = BASE_FRAME_ID;
    virtual_octomap.header.stamp = ros::Time::now();
    virtual_octomap_pub.publish(virtual_octomap);
}

void OctomapExploration::publish_frontier_text_labels(std::vector<double> frontier_mi,
        std::vector<octomap::point3d> frontier_pos){

    if(frontier_mi.size() != frontier_pos.size()){
        ROS_ERROR_STREAM("frontier mi is different in size than frontier pos (" << frontier_mi.size()
        << " != " <<  frontier_pos.size() << ")");
        return;
    }

    visualization_msgs::MarkerArray frontier_mi_labels;

    for(std::vector<double>::size_type i = 0; i != frontier_mi.size(); i++) {
        double mi = frontier_mi[i];
        octomap::point3d origin = frontier_pos[i];
        origin = octomap::point3d(origin.x(), origin.y(), origin.z() + 1.0);

        // prepare text labels
        std::stringstream entropy_label;
        entropy_label << "mi:" << std::fixed << std::setprecision(3) << mi;

        frontier_mi_labels.markers.push_back(
                prepareMarkerROS(
                        origin,
                        i,
                        visualization_msgs::Marker::TEXT_VIEW_FACING,
                        entropy_label.str(),
                        octomap::point3d(0.8, 0.8, 0.8),
                        1.0,
                        1.0,
                        1.0,
                        1.0,
                        BASE_FRAME_ID,
                        120
                ));
    }

    frontier_mi_label_pub.publish(frontier_mi_labels);
}

std::vector<double> OctomapExploration::process_octomap(octomap::OcTree *octomap_base_param) {
    std::vector<double> frontier_mi;
    std::vector<octomap::point3d> frontier_pos;

    if (octomap_base_param == NULL) {
        ROS_ERROR("Octomap is empty, wait for node to publish %s",
                  OCTOMAP_BINARY_TOPIC.c_str());
        return frontier_mi;
    }

    if (frontier_centers_array == NULL) {
        ROS_ERROR("frontier_centers_array is empty, wait for node to publish %s",
                  FRONTIER_CENTERS_MARKERS_TOPIC.c_str());
        return frontier_mi;
    }

    int frontier_size = frontier_centers_array->markers.size();
    ROS_INFO("Frontier size: %d", frontier_size);
    if (frontier_size <= 0) {
        ROS_ERROR("frontier_centers_array is empty");
        return frontier_mi;
    }

    double octomap_res = octomap_base_param->getResolution();
    auto octomap_curr = new octomap::OcTree(*octomap_base_param);

    ROS_INFO_STREAM("occupancy thresh: " << octomap_load->getOccupancyThres());
    ROS_INFO_STREAM("loaded octomap free volume: " << get_free_volume(octomap_base_param));
    ROS_INFO_STREAM("new octomap free volume: " << get_free_volume(octomap_curr));

    #pragma omp parallel for
    // testing and speedup
    for (int i = 0; i < frontier_size; ++i) {
        double x1 = frontier_centers_array->markers[i].pose.position.x;
        double y1 = frontier_centers_array->markers[i].pose.position.y;
        double z1 = frontier_centers_array->markers[i].pose.position.z;
        z1 += 0.5; // simulate ouster tf TODO: check this tf
        octomap::point3d origin(x1, y1, z1);

        // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
        tf::Quaternion quat;
        tf::quaternionMsgToTF(frontier_centers_array->markers[i].pose.orientation, quat);

        // the tf::Quaternion has a method to acess roll pitch and yaw
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        octomap::point3d direction(roll, pitch, yaw);

        double mi_before = get_free_volume(octomap_curr);

        octomap::Pointcloud hits = cast_rays(octomap_curr, origin, direction);

        double mi_after = calc_MI(octomap_curr, origin, hits, mi_before);

        ROS_DEBUG_STREAM("Candidate: " << i << " original MI:" << mi_before << " MI after:" << mi_after <<
        " origin:" << origin);

        frontier_mi.push_back(mi_after);
        frontier_pos.push_back(origin);
        ROS_DEBUG_STREAM(" <<<< ------ <<<< " << i + 1 << "/" << frontier_size);

//        ros::Duration(2.0).sleep();
//        std::string inputString;
//        std::cout << "Give input";
//        std::cin.clear();
//        std::cin.ignore(INT_MAX,'\n');
//        std::getline(std::cin, inputString);
    }

    publish_virtual_octomap(octomap_curr);

//    double x1 = 362.816040039;
//    double y1 = 74.552520752;
//    double z1 =  -5.90169763565;
//    z1 += 0.5; // simulate ouster tf TODO: check this tf
//    octomap::point3d origin(x1, y1, z1);
//    tf::Quaternion quat;
//    octomap::point3d direction(0, 0, 0);
//    double mi_before = get_free_volume(octomap_curr);
//    octomap::Pointcloud hits = cast_rays(octomap_curr, origin, direction);
//    double mi_after = calc_MI(octomap_curr, origin, hits, mi_before);
//    frontier_mi.push_back(mi_after);
//    frontier_pos.push_back(origin);
//    ROS_DEBUG_STREAM("Candidate: LAST" << " original MI:" << mi_before << " MI after:" << mi_after <<
//        " origin:" << origin);

    ROS_DEBUG_STREAM("deleting octomap curr");
    delete octomap_curr;
    octomap_curr = NULL;

    publish_frontier_text_labels(frontier_mi, frontier_pos);
    return frontier_mi;
}


bool OctomapExploration::processFrontierByIdSrvCallback(espeleo_planner::processFrontier::Request &req,
                                    espeleo_planner::processFrontier::Response &res) {

    ROS_INFO_STREAM("processFrontier by ID srv called");

    // TODO: add all functions here

    return true;
}

bool OctomapExploration::processAllFrontiersSrvCallback(espeleo_planner::processAllFrontiers::Request &req,
                                    espeleo_planner::processAllFrontiers::Response &res) {

    ROS_INFO_STREAM("processAllFrontiers srv called");

    std::vector<double> frontier_mi;
    try {
        std::unique_lock<std::mutex> lock(octomap_mutex);
        std::vector<double> frontier_mi = process_octomap(octomap_load);
        res.mi_data = frontier_mi;
        res.success = true;
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception when processing octomap:" << e.what());
        if (octomap_load != NULL) {
            ROS_DEBUG_STREAM("exception octomap_load is not NULL, cleaning octomap_load");
            delete octomap_load;
            octomap_load = NULL;
        }

        res.success = false;
        return false;
    }

    if (octomap_load != NULL) {
        ROS_DEBUG_STREAM("octomap_load is not NULL, cleaning octomap_load");
        delete octomap_load;
        octomap_load = NULL;
    }

    //ROS_INFO_STREAM("sending back response:" << res.mi_data);
    return true;
}

bool OctomapExploration::processAllFrontiersUsingSTLOctomapSrvCallback(
        espeleo_planner::processAllFrontiersUsingSTLOctomap::Request &req,
        espeleo_planner::processAllFrontiersUsingSTLOctomap::Response &res) {

    ROS_INFO_STREAM("processAllFrontiers srv called");

    if (octomap_load == NULL) {
        ROS_ERROR("octomap_load is empty, wait for node to publish %s",
                  OCTOMAP_BINARY_TOPIC.c_str());
        res.success = false;
        return false;
    }

    std::unique_lock<std::mutex> lock(octomap_mutex);

    std::vector<double> frontier_mi;
    try {
        string stl_path = req.stl_path;
        string binvox_cmd = "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/test/binvox -d 256 -e " + stl_path;
        ROS_INFO_STREAM("executed binvox cmd: " << binvox_cmd);
        std::system(binvox_cmd.c_str());

        string binvox_path = boost::filesystem::change_extension(stl_path, "binvox").string();
        ROS_INFO_STREAM("binvox_path: " << binvox_path);

        string binvox2bt_cmd = "/home/h3ct0r/catkin_ws_espeleo/devel/lib/espeleo_planner/binvox2bt " + binvox_path;
        ROS_INFO_STREAM("binvox_path: " << binvox2bt_cmd);
        std::system(binvox2bt_cmd.c_str());

        string octree_path = binvox_path + ".bt";
        ROS_INFO_STREAM("octree_path: " << octree_path);

        octomap_from_stl = new octomap::OcTree(octree_path);
        ROS_INFO_STREAM("octree size: " << octomap_from_stl->size() << " resolution:" << octomap_from_stl->getResolution());

        ROS_DEBUG_STREAM("Merging octomaps...");
        // Expand tree2 so we search all nodes
        octomap_from_stl->expand();
        // traverse nodes in tree2 to add them to tree1
        for (octomap::OcTree::leaf_iterator it = octomap_from_stl->begin_leafs();
             it != octomap_from_stl->end_leafs(); ++it) {

            // find if the current node maps a point in map1
            octomap::point3d point = it.getCoordinate();
            //ROS_DEBUG_STREAM("coordinate...");
            octomap::OcTreeNode *nodeIn1 = octomap_load->search(point);
            //ROS_DEBUG_STREAM("nodeIn1...");
            if (nodeIn1 != NULL) {
                //ROS_DEBUG_STREAM("!= null...");
                // Add the probability of tree2 node to the found node
                octomap::OcTreeKey nodeKey = octomap_load->coordToKey(point);
                octomap_load->updateNode(nodeKey, it->getLogOdds());
            } else {
                //ROS_DEBUG_STREAM("== null...");
                // Create a new node and set the probability from tree2
                octomap::OcTreeNode *newNode = octomap_load->updateNode(point, true);
                newNode->setLogOdds(it->getLogOdds());
            }
        }
        ROS_DEBUG_STREAM("Merging octomaps DONE");

        std::vector<double> frontier_mi = process_octomap(octomap_load);
        res.mi_data = frontier_mi;
        res.success = true;
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception when processing octomap:" << e.what());
        if (octomap_from_stl != NULL) {
            ROS_DEBUG_STREAM("exception octomap_from_stl is not NULL, cleaning octomap_from_stl");
            delete octomap_from_stl;
            octomap_from_stl = NULL;
        }

        if (octomap_load != NULL) {
            ROS_DEBUG_STREAM("exception octomap_load is not NULL, cleaning octomap_load");
            delete octomap_load;
            octomap_load = NULL;
        }

        res.success = false;
        return false;
    }

    //ROS_INFO_STREAM("sending back response:" << res.mi_data);
    if (octomap_from_stl != NULL) {
        ROS_DEBUG_STREAM("octomap_from_stl is not NULL, cleaning octomap_from_stl");
        delete octomap_from_stl;
        octomap_from_stl = NULL;
    }

    if (octomap_load != NULL) {
        ROS_DEBUG_STREAM("octomap_load is not NULL, cleaning octomap_load");
        delete octomap_load;
        octomap_load = NULL;
    }

    return true;
}

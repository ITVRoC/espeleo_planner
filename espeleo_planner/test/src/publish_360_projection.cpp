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

using namespace std;
using namespace std::chrono;

typedef octomap::point3d point3d;
const double PI = 3.1415926;
const double free_prob = 0.3;

struct FrontierInformation {
    double entropy = std::numeric_limits<double>::infinity();
    boost::tuple<double, double, double> center;
    std::chrono::milliseconds last_updated_ms = duration_cast< milliseconds >(
        system_clock::now().time_since_epoch()
    );

    FrontierInformation(boost::tuple<double, double, double> _center)
        : center(_center){
        
       // do something
    }

    bool isEntropyCalculated(){
        return entropy == std::numeric_limits<double>::infinity();
    }

};

struct Kinect {

    double horizontal_fov;
    double angle_inc;
    double width;
    double height;
    double max_range;
    vector<pair<double, double>> pitch_yaws;

    Kinect(double _width, double _height, double _horizontal_fov, double _max_range)
        : width(_width), height(_height), horizontal_fov(_horizontal_fov), max_range(_max_range) {
        
        angle_inc = horizontal_fov / width;
        for(double i = -width / 2; i < width / 2; ++i) {
            for(double j = -height / 2; j < height / 2; ++j) {
                pitch_yaws.push_back(make_pair(j * angle_inc, i * angle_inc));
            }
        }
    }
};

//Kinect kinect(640, 480, 1.047198, 4.0);
Kinect kinect(80, 60, 1.047198, 4.0);

class PointCloudPub {
    typedef pcl::PointXYZ PointType;
    typedef pcl::PointCloud<PointType> PointCloud;

public:
    PointCloudPub(ros::NodeHandle _nh) : nh(_nh), msg(new PointCloud), tp_name("virtualScans") {
        msg->header.frame_id = pointcloud_frame_id;
        msg->height = 1;
        msg->width = 0;
        // pub = nh.advertise<PointCloud>("virtualScans", 1);
        pub = nh.advertise<PointCloud>(tp_name, 1);
    }

    void SetTopicName(string tp_name) {
        pub = nh.advertise<PointCloud>(tp_name, 1);
    }

    void insert_point3d(double x, double y, double z) {
        msg->points.push_back(PointType(x, y, z));
        msg->width++;
    }

    void clear() {
        msg->points.clear();
        msg->width = 0;
    }

    void publish() const {
        msg->header.stamp = ros::Time::now().toNSec() / 1e3;
        pub.publish(msg);
        // cout << "published : " << msg->width << " points" << endl;
        ros::spinOnce();
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    const string pointcloud_frame_id = "/map";
    PointCloud::Ptr msg;
    string tp_name;
};

class OctomapExploration {
public:
    OctomapExploration(ros::NodeHandle _nh) : nh(_nh), pointcloud_pub(_nh), CurrentPcl_pub(_nh) {
        position = point3d(0, 0, 0);
        orientation = point3d(0, 0, 0);
        octomap_sub = nh.subscribe<octomap_msgs::Octomap>(OCTOMAP_BINARY_TOPIC, 1,
                      &OctomapExploration::octomap_callback, this);

        frontier_centers_sub = nh.subscribe<visualization_msgs::MarkerArray>(FRONTIER_CENTERS_MARKERS_TOPIC, 1,
                      &OctomapExploration::frontier_centers_callback, this);

        //process_frontier_srv = nh.advertiseService(PROCESS_FRONTIER_BY_ID_SRV_NAME, &OctomapExploration::processFrontierByIdSrvCallback, this);
        //process_all_frontiers_srv = nh.advertiseService(PROCESS_ALL_FRONTIERS_SRV_NAME, &OctomapExploration::processAllFrontiersSrvCallback, this);
    }

    ~OctomapExploration() {
    }

    void octomap_callback(const octomap_msgs::Octomap::ConstPtr &msg) {
        ROS_INFO("octomap_callback");
        octomap_load = dynamic_cast<octomap::OcTree *>(octomap_msgs::msgToMap(*msg));
    }

    void frontier_centers_callback(const visualization_msgs::MarkerArray::ConstPtr& msg){
        ROS_INFO("frontier_marker_callback");
        frontier_centers_array = msg;

        int frontier_size = frontier_centers_array->markers.size();
        ROS_INFO("Frontier size: %d", frontier_size);
        if(frontier_size <= 0) {
            ROS_ERROR("frontier_centers_array is empty, not updating frontierInfo map");
            return;
        }

        std::chrono::milliseconds current_ms = duration_cast< milliseconds >(
            system_clock::now().time_since_epoch()
        );

        // iterate all the frontiers received
        for(int i = 0; i < frontier_size; ++i) {
            double x1 = frontier_centers_array->markers[i].pose.position.x;
	        double y1 = frontier_centers_array->markers[i].pose.position.y;
	        double z1 = frontier_centers_array->markers[i].pose.position.z;
            z1 += 0.5; // simulate Xtion position that is half meter over the groudn frontier center TODO: use the real TF for this!
            boost::tuple<double, double, double> p1_boost = boost::make_tuple(x1, y1, z1);

            // if this frontier is already in the map
            // then only update the last updated time
            auto it = frontierInfo.find(p1_boost);
            if(it != frontierInfo.end()){
                it->second.last_updated_ms = current_ms;
                continue;
            }

            // if is not in the map then add it and update the last updated time
            auto fInfo = FrontierInformation(p1_boost);
            fInfo.last_updated_ms = current_ms;
            frontierInfo.insert(std::pair<boost::tuple<double, double, double>, FrontierInformation>(p1_boost, fInfo));
        }

        // remove old frontiers by last_updated_ms
        for (auto it = frontierInfo.cbegin(); it != frontierInfo.cend();){
            if(it->second.last_updated_ms != current_ms){
                ROS_DEBUG_STREAM("[DEBUG] removing frontier:" << convert3DTupleToPoint3D(it->first) << "\t" << it->second.entropy);
                frontierInfo.erase(it++);    // or "it = m.erase(it)" since C++11
            }
            else {
                ++it;
            }
        }

        // debug
        for (auto it = frontierInfo.begin(); it != frontierInfo.end(); ++it) { 
            ROS_DEBUG_STREAM("[DEBUG] frontierInfo:" << convert3DTupleToPoint3D(it->first) << "\t" << it->second.entropy); 
        }
    }

    octomap::point3d convert3DTupleToPoint3D(boost::tuple<double, double, double> p1_boost){
        return octomap::point3d(
            p1_boost.get<0>(),
            p1_boost.get<1>(),
            p1_boost.get<2>()
        );
    }

    boost::tuple<double, double, double> convertPoint3Dto3DTuple(octomap::point3d p1){
        return boost::make_tuple((double) p1.x(), (double) p1.y(), (double) p1.z());
    }

    vector<point3d> cast_kinect_rays(const octomap::OcTree *octree, const point3d &position,
                                     const point3d &direction) const {
        vector<point3d> hits;
        octomap::OcTreeNode *n;

        //ROS_DEBUG_STREAM("d_x:" << direction.normalized().x() << " d_y:" << direction.normalized().y() << " d_z:"  << direction.normalized().z());
        //ROS_DEBUG_STREAM("px:" << position.x() << " py:" << position.y() << " pz:"  << position.z());
        //ROS_DEBUG_STREAM("dx:" << direction.x() << " dy:" << direction.y() << " dz:"  << direction.z());
        for(auto p_y : kinect.pitch_yaws) {
            double pitch = p_y.first;
            double yaw = p_y.second;
            point3d direction_copy(direction);
            point3d end;

            //ROS_DEBUG_STREAM("pitch:" << pitch << " yaw:" << yaw << " position:" << position << " direction_copy:" << direction_copy);

            direction_copy.rotate_IP(0, pitch, yaw);

            // direction_copy += position;
            // hits.push_back(direction_copy);
            //ROS_DEBUG_STREAM("direction_copy:" << direction_copy);

            

            if(octree->castRay(position, direction_copy, end, true, kinect.max_range)) {
                // remove points inside the known section of the map
                hits.push_back(end);
            } else {
                //direction_copy *= kinect.max_range;
                direction_copy += position;
                n = octree->search(direction_copy);
                // if (!n)
                //     continue;
                // if (n->getOccupancy() < free_prob )
                //     continue;

                // check if proyected point is way bellow the current position
                if(direction_copy(2) < (position(2) - 0.2))
                    continue;

                hits.push_back(direction_copy);
            }
        }
        return hits;
    }

    double get_free_volume(const octomap::OcTree *octree) const {
        double volume = 0;
        for(octomap::OcTree::leaf_iterator n = octree->begin_leafs(octree->getTreeDepth()); n != octree->end_leafs(); ++n) {
            if(!octree->isNodeOccupied(*n))
                volume += pow(n.getSize(), 3);
            // cout << "node : " << n.getCoordinate() << endl;
        }
        return volume;
    }

    double calc_MI(octomap::OcTree *octree, const point3d &position, const vector<point3d> &hits, const double before) const {
        
        double after = get_free_volume(octree);
        float free_logodds = octree->getClampingThresMinLog();

        for(point3d h : hits) {
            octree->insertRay(position, h, kinect.max_range);
            // mark the end of the ray as empty
            for(octomap::OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(h, h), end=octree->end_leafs_bbx(); it!= end; ++it){
                // directly set values of leafs:
                it->setLogOdds(free_logodds);
            }
            //integrateLocalMissOnRay(octree, position, h, m_keyRay, false, kinect.max_range);
        }

        octree->updateInnerOccupancy();
        after = get_free_volume(octree);
        
        return after - before;
    }

    // void integrateLocalMissOnRay(octomap::OcTree *octree, const point3d& origin, point3d& end, octomap::KeyRay m_keyRay, bool lazy_eval, float max_range) const {

    //     // instead of direct scan insertion, compute update for virtual "sensing" of the empty space
    //     // based on OctomapServer::insertScan 
    //     // (https://github.com/OctoMap/octomap_mapping/blob/kinetic-devel/octomap_server/src/OctomapServer.cpp)

    //     octomap::KeySet free_cells;
        
    //     if ((max_range > 0.0) && ((end - origin).norm() > max_range)) {
    //         end = origin + (end - origin).normalized() * max_range;
    //     }

    //     if (octree->computeRayKeys(origin, end, m_keyRay)){
    //         free_cells.insert(m_keyRay.begin(), m_keyRay.end());
    //     }

    //     // mark free cells only if not seen occupied in this cloud
    //     for(octomap::KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it){
    //         octree->updateNode(*it, false, lazy_eval);
    //     }

    //     // if (!octree->computeRayKeys(origin, end, octree->keyrays.at(0))) {
    //     //     return false;
    //     // }

    //     // for(octomap::KeyRay::iterator it=octree->keyrays[0].begin(); it != octree->keyrays[0].end(); it++) {s
    //     //     updateNode(*it, false, lazy_eval); // insert freespace measurement
    //     // }

    //     // return true;
    // }

    visualization_msgs::MarkerArray process_octomap() {
        visualization_msgs::MarkerArray frontier_MI_labels;
        frontier_MI_labels.markers.clear();

        if(octomap_load == NULL) {
            ROS_ERROR("Octomap is empty, wait for node to publish %s", 
                OCTOMAP_BINARY_TOPIC.c_str());
            return frontier_MI_labels;
        }

        if(frontier_centers_array == NULL) {
            ROS_ERROR("frontier_centers_array is empty, wait for node to publish %s", 
                FRONTIER_CENTERS_MARKERS_TOPIC.c_str());
            return frontier_MI_labels;
        }

        int frontier_size = frontier_centers_array->markers.size();
        ROS_INFO("Frontier size: %d", frontier_size);
        if(frontier_size <= 0) {
            ROS_ERROR("frontier_centers_array is empty");
            return frontier_MI_labels;
        }

        double octomap_res = octomap_load->getResolution();
        // octomap::OcTree tree(octomap_res);
        // octomap::OcTree *octomap_curr = &tree;

        octomap::OcTree *octomap_curr(octomap_load);

        ROS_INFO_STREAM("occupancy thresh: " << octomap_load->getOccupancyThres());
        ROS_INFO_STREAM("loaded octomap free volume: " << get_free_volume(octomap_load));
        ROS_INFO_STREAM("new octomap free volume: " << get_free_volume(octomap_curr));

        vector<double> mutual_entropies(frontier_size);
        octomap::OcTreeNode *n;
        double bbox_offset = 0.5;

        #pragma omp parallel for
        // testing and speedup
        // frontier_size
        // 1
        for(int i = 0; i < frontier_size; ++i) {
            double x1 = frontier_centers_array->markers[i].pose.position.x;
	        double y1 = frontier_centers_array->markers[i].pose.position.y;
	        double z1 = frontier_centers_array->markers[i].pose.position.z;
            z1 += 0.5; // simulate kinetic tf
            octomap::point3d p1(x1, y1, z1);

            double roll = 0;
            double pitch = 0;
            double yaw = 0;

            // n = octomap_load->search(x1, y1, z1);
            // if(!n) {
            //     ROS_ERROR("octomap_load->search() false (%f, %f, %f)", x1, y1, z1);
            //     continue;
            // }

            // if(n->getOccupancy() > free_prob) {
            //     ROS_ERROR_STREAM("occupancy " << n->getOccupancy());
            //     continue;
            // }

            

            double mi_before = get_free_volume(octomap_curr);

            // visualization_msgs::MarkerArray bb_array;
            // bb_array.markers.clear();

            ROS_DEBUG_STREAM("Casting rays...");
            vector<point3d> hits;
            high_resolution_clock::time_point t1 = high_resolution_clock::now();
            for (int angle_i=0; angle_i < 360; angle_i += 60) {
                octomap::point3d rotating(kinect.max_range, 0.00, 0.00);
                rotating.rotate_IP(0, 0, DEG2RAD(angle_i));
                vector<point3d> l_hits = cast_kinect_rays(octomap_curr, p1, rotating);

                hits.reserve(l_hits.size());
                hits.insert(hits.end(), l_hits.begin(), l_hits.end());
            }
            high_resolution_clock::time_point t2 = high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
            ROS_DEBUG_STREAM("Time on ray cast: " << duration << "ms");
            ROS_DEBUG_STREAM("\thits: "<< hits.size());

            // ROS_DEBUG_STREAM("drawing markers");
            // int marker_id = 0;
            // for(int idx=0; idx < hits.size(); idx += 1000){
            //     visualization_msgs::Marker marker;
            //     marker.header.frame_id = "/map"; 
            //     marker.header.stamp = ros::Time::now();
            //     //marker.ns = "bb_test";
            //     marker.id = marker_id;

                
            //     uint32_t shape = visualization_msgs::Marker::SPHERE;
            //     marker.type = shape;

            //     marker.pose.position.x = hits[idx](0);
            //     marker.pose.position.y = hits[idx](1);
            //     marker.pose.position.z = hits[idx](2);

            //     //ROS_DEBUG_STREAM("hit id: " << idx << " " << hits[idx]);

            //     marker.scale.x = 0.3;
            //     marker.scale.y = 0.3;
            //     marker.scale.z = 0.3;

            //     marker.color.r = 0.1;
            //     marker.color.g = 0.4;
            //     marker.color.b = 1.0;
            //     marker.color.a = 0.85;

            //     bb_array.markers.push_back(marker);
            //     marker_id ++;
            // }
            //kinetic_marker_pub.publish(bb_array);
            // ROS_DEBUG_STREAM("published bb_array.markers: " << bb_array.markers.size());

            t1 = high_resolution_clock::now();
            auto octree_copy = new octomap::OcTree(*octomap_curr);
            double mi_after = calc_MI(octree_copy, p1, hits, mi_before);
            t2 = high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
            ROS_DEBUG_STREAM("Time on estimating MI: " << duration << "ms");

            t1 = high_resolution_clock::now();
            octomap_msgs::Octomap virtual_octomap;
            bool res = octomap_msgs::binaryMapToMsg(*octree_copy, virtual_octomap);
            virtual_octomap.header.frame_id = "/map";
	        virtual_octomap.header.stamp = ros::Time::now();
            ROS_DEBUG_STREAM("Convert to binaryMapMsg res:" << res);
            virtual_octomap_pub.publish(virtual_octomap);
            delete(octree_copy);
            t2 = high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
            ROS_DEBUG_STREAM("Time publising updated octomap: " << duration << "ms");

            ROS_DEBUG_STREAM("Candidate: " << p1 << " original MI:" << mi_before << " MI after:" << mi_after);

            // prepare text labels
            std::stringstream entropy_label;
            entropy_label << std::fixed << std::setprecision(3) << mi_after;
            octomap::point3d p_label = octomap::point3d(p1.x(), p1.y(), p1.z() + 1.0);
		    frontier_MI_labels.markers.push_back(
                prepareMarkerROS(
                    p_label,
                    i,
                    visualization_msgs::Marker::TEXT_VIEW_FACING,
                    "mi:" + entropy_label.str(),
                    octomap::point3d(1.0, 1.0, 0.5),
                    1.0,
                    frontier_centers_array->markers[i].color.r,
                    frontier_centers_array->markers[i].color.g,
                    frontier_centers_array->markers[i].color.b
                ));
            ROS_DEBUG_STREAM(" <<<< ------ <<<< " << i+1 << "/" <<  frontier_size);
        }
        frontier_MI_label_pub.publish(frontier_MI_labels);

        return frontier_MI_labels;
    }

    visualization_msgs::Marker prepareMarkerROS(octomap::point3d origin, int id, uint32_t shape, std::string label, 
        octomap::point3d scale, float a, float r, float g, float b) {
            visualization_msgs::Marker marker;
			marker.header.frame_id = "/map"; 
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

            marker.lifetime = ros::Duration(15.0);

            return marker;
    }

//    bool processFrontierByIdSrvCallback(heterogeneus_frontier_exploration::processFrontier::Request &req,
//        heterogeneus_frontier_exploration::processFrontier::Response &res){
//
//        ROS_INFO_STREAM("processFrontier by ID srv called");
//
//        // TODO: add all functions here
//
//        return true;
//    }
//
//    bool processAllFrontiersSrvCallback(heterogeneus_frontier_exploration::processAllFrontiers::Request &req,
//        heterogeneus_frontier_exploration::processAllFrontiers::Response &res){
//
//        ROS_INFO_STREAM("processAllFrontiers srv called");
//
//        visualization_msgs::MarkerArray frontier_MI_labels = process_octomap();
//        res.frontiersMI = frontier_MI_labels;
//        res.success = true;
//        ROS_INFO_STREAM("sending back response:" << res.frontiersMI);
//
//        return true;
//    }

    

private:
    ros::NodeHandle nh;
    const string OCTOMAP_BINARY_TOPIC = "/octomap_binary";
    const string FRONTIER_CENTERS_MARKERS_TOPIC = "/frontiers_ground_centers";
    const string PROCESS_FRONTIER_BY_ID_SRV_NAME = "/process_frontier_by_id";
    const string PROCESS_ALL_FRONTIERS_SRV_NAME = "/process_all_frontiers";
    ros::Subscriber octomap_sub;
    ros::Subscriber frontier_centers_sub;

    PointCloudPub pointcloud_pub;
    PointCloudPub CurrentPcl_pub;

    ros::Publisher virtual_octomap_pub = nh.advertise<octomap_msgs::Octomap>("/virtual_octomap", 1);
    ros::Publisher kinetic_marker_pub = nh.advertise<visualization_msgs::MarkerArray> ("/test_kinetic_FOV", 0);
    ros::Publisher frontier_MI_label_pub = nh.advertise<visualization_msgs::MarkerArray> ("/frontiers_MI", 0);

    ros::ServiceServer process_frontier_srv;
    ros::ServiceServer process_all_frontiers_srv;

    octomap::OcTree *octomap_load;
    octomap_msgs::Octomap::ConstPtr octomap_binary;
    visualization_msgs::MarkerArray::ConstPtr frontier_centers_array;
    octomap::point3d position;
    octomap::point3d orientation;
    std::map<boost::tuple<double, double, double>, FrontierInformation> frontierInfo; 
};

int main(int argc, char **argv) {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::console::notifyLoggerLevelsChanged();

    ros::init(argc, argv, "octomap_MI");
    ros::NodeHandle nh;
    OctomapExploration octomap_exploration(nh);

    ros::Duration(2.0).sleep();
    ros::Rate r(5); // 1 hz
    while (ros::ok())
    {   
        //octomap_exploration.process_octomap(); // TODO: this should be a service! testing
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}

/*
CurrentPcl_pub.clear();
        for(octomap::OcTree::leaf_iterator n = octomap_curr->begin_leafs(octomap_curr->getTreeDepth()); n != octomap_curr->end_leafs(); ++n) {
            if(octomap_curr->isNodeOccupied(*n))
                CurrentPcl_pub.insert_point3d(n.getCoordinate().x()/5.0,
                                                n.getCoordinate().y()/5.0, n.getCoordinate().z()/5.0 );
        }
        CurrentPcl_pub.publish();
*/
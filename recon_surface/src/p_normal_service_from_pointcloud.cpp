#include <ros/ros.h>
#include <fstream>

#include "recon_surface/NormalsFromPointCloud2.h"

#include <iostream>
#include "recon_surface/surface_recon.hpp"
#include "recon_surface/pset.hpp"
#include "recon_surface/hole_filling.hpp"
#include "recon_surface/rm_artifacts.hpp"
#include "recon_surface/parser.hpp"
#include "recon_surface/texture.hpp"

#include <dynamic_reconfigure/server.h>
#include <recon_surface/ReconSurfaceConfig.h>

#include <map>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>

#include <iostream>
#include <iomanip>
#include <ctime>
#include <CGAL/Vector_3.h>

#include <CGAL/Kernel_traits.h>
#include <CGAL/boost/graph/properties.h>

#include <boost/cstdint.hpp>
#include <boost/graph/graph_traits.hpp>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <iostream>
#include <chrono>
#include <visualization_msgs/Marker.h>

double gridm = 0.0;
double holemaxsize = 0.0;
string output_folder = "";
double sample = 0.0;
double trim = 0.0;
int texturesize = 0;

string generate_point_cloud_normals(const sensor_msgs::PointCloud2 msg, const geometry_msgs::Point src, string output) {
    std::vector<Point> points;
    std::vector<Vector> normals;
    std::vector<Color> colors;
    std::list<PointVectorPair> estimated_pwn;
    std::vector<Point_3> orig_points;
    std::vector<unsigned int> indices;
    auto src_point = Point_3(src.x, src.y, src.z);

    FT average_spacing = 0.;

    auto start_total_time = chrono::high_resolution_clock::now();

    auto start = chrono::high_resolution_clock::now();
    Pset pointset(points, normals, colors, sample); // init my point cloud
    pointset.read_pointCloud2(msg);

    for (size_t i = 0; i < points.size(); i++) //initialize structure for Kdtree
    {
        orig_points.push_back(Point_3(points[i][0], points[i][1], points[i][2]));
        indices.push_back(i);
    }
    auto stop = chrono::high_resolution_clock::now();

    ROS_INFO_STREAM("Time initialize pointcloud: "
                            << float(chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0)
                            << " seconds");

    // Initialize a spatial kd-tree for further search on original set of points
    start = chrono::high_resolution_clock::now();
    Tree tree(
            boost::make_zip_iterator(boost::make_tuple(orig_points.begin(), indices.begin())),
            boost::make_zip_iterator(boost::make_tuple(orig_points.end(), indices.end()))
    );

    pointset.write_ply(output + "_full_pcd_rgb.ply"); //write full rgb .ply
    stop = chrono::high_resolution_clock::now();
    ROS_INFO_STREAM("Time to generate full rgb *.ply file: "
                            << float(chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0)
                            << " seconds");

    start = chrono::high_resolution_clock::now();
    average_spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>(orig_points.begin(), orig_points.end(), 6);
    stop = chrono::high_resolution_clock::now();
    ROS_INFO_STREAM("Time to compute_average_spacing: "
                            << float(chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0)
                            << " seconds");

    double as_ratio = gridm * log(points.size() / 10e4);

    if (as_ratio < 0.6)
        as_ratio = 0.6;

    double cell_size = average_spacing * as_ratio;

    ROS_INFO_STREAM("Estimated weight for grid size w.r.t. avg. point distance: " << as_ratio);

    start = chrono::high_resolution_clock::now();
    pointset.sample_points_cgal(cell_size, 1);
    stop = chrono::high_resolution_clock::now();
    ROS_INFO_STREAM("Time to sample_points_cgal: "
                            << float(chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0)
                            << " seconds");

    ROS_INFO_STREAM(
            "Sampled point set size: " << points.size() << " | Normals: " << normals.size() << " | Original ply size: "
                                       << colors.size());

    start = chrono::high_resolution_clock::now();
    if (normals.size() == 0){
        ROS_INFO_STREAM("Estimating normals");
        estimate_normals(points, src_point, estimated_pwn);
    }
    else{
        ROS_INFO_STREAM("Registering normals normals");
        estimated_pwn = register_normals(points, grab_normals(orig_points, normals));
    }

    stop = chrono::high_resolution_clock::now();
    ROS_INFO_STREAM("Time to estimate normals: "
                            << float(chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0)
                            << " seconds");

    ROS_INFO_STREAM("Points sizes:" << points.size() << " Estimated PWN size:" << estimated_pwn.size());

    start = chrono::high_resolution_clock::now();
    string output_ply_filepath = output + "_simplified_pcd_normals.ply";
    //write_ply_wnormals(output_ply_filepath, estimated_pwn, tree, colors);
    write_ply_binary_wnormals(output_ply_filepath, estimated_pwn, tree, colors);

    stop = chrono::high_resolution_clock::now();
    ROS_INFO_STREAM("Time to write point cloud .ply file: "
                            << float(chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0)
                            << " seconds");
    ROS_INFO_STREAM("PLY file written to:" << output_ply_filepath);

//    start = chrono::high_resolution_clock::now();
//    trim_mesh(reconstruct_surface(estimated_pwn, output), tree, trim * (double) average_spacing, output);
//    stop = chrono::high_resolution_clock::now();
//    ROS_INFO_STREAM("Time trim mesh: "
//                            << float(chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0)
//                            << " seconds");
//
//    start = chrono::high_resolution_clock::now();
//    fill_hole(output + "temp3.off", holemaxsize * (double) average_spacing, output);
//    stop = chrono::high_resolution_clock::now();
//    ROS_INFO_STREAM("Time fill holes: "
//                            << float(chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0)
//                            << " seconds");
//
////    start = chrono::high_resolution_clock::now();
////    texture t(tree, colors, output + "_textured.obj", texturesize);
////    t.build_png();
////    t.save_obj();
////    stop = chrono::high_resolution_clock::now();
////    ROS_INFO_STREAM("Time generate *.obj file: "
////                            << float(chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0)
////                            << " seconds");
//
////    ROS_INFO_STREAM("Writing STL");
//
//    std::ifstream input_off3(output + "temp3.off");
//    string output_stl_filepath = output + "_mesh.stl";
//    string output_stl_filepath_inverted = output + "_inverted_mesh.stl";
//    Mesh mesh_obj;
//
//
//    stop = chrono::high_resolution_clock::now();
//    ROS_INFO_STREAM("Time total reconstruction: "
//                            << float(chrono::duration_cast<chrono::microseconds>(stop - start_total_time).count() / 1000000.0)
//                            << " seconds");

//    if (!input_off3 || !(input_off3 >> mesh_obj)) {
//        ROS_ERROR_STREAM("Not a valid off file (generate_mesh): " << output_stl_filepath);
//        return "";
//    } else {
//        start = chrono::high_resolution_clock::now();
//
//        writeSTLfromMesh(mesh_obj, output_stl_filepath, false, true);
//        publishSTLMarker(marker_pub_normal, output_stl_filepath, msg.header.frame_id);
//
//        writeSTLfromMesh(mesh_obj, output_stl_filepath_inverted, true, true);
//        publishSTLMarker(marker_pub_inverted, output_stl_filepath_inverted, msg.header.frame_id);
//
//        ROS_INFO_STREAM("STL files written to:" << output_stl_filepath << " and " << output_stl_filepath_inverted);
//        stop = chrono::high_resolution_clock::now();
//        ROS_INFO_STREAM("Time generate *.stl file: "
//                                << float(chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0)
//                                << " seconds");
//    }

    return output_ply_filepath;
}

void dynReconfigureCallback(recon_surface::ReconSurfaceConfig &config, uint32_t level) {
    if (gridm != config.gridm) {
        gridm = config.gridm;
        ROS_INFO("Reconfigure gridm: %lf", gridm);
    }

    if (holemaxsize != config.holemaxsize) {
        holemaxsize = config.holemaxsize;
        ROS_INFO("Reconfigure holemaxsize: %lf", holemaxsize);
    }

    if (output_folder != config.output_folder) {
        output_folder = config.output_folder;
        ROS_INFO("Reconfigure output_folder: %s", output_folder.c_str());
    }

    if (sample != config.sample) {
        sample = config.sample;
        ROS_INFO("Reconfigure sample: %lf", sample);
    }

    if (texturesize != config.texturesize) {
        texturesize = config.texturesize;
        ROS_INFO("Reconfigure texturesize: %lf", texturesize);
    }

    if (trim != config.trim) {
        trim = config.trim;
        ROS_INFO("Reconfigure trim: %lf", trim);
    }
}

bool pointNormalsFromPointCloudCallback(recon_surface::NormalsFromPointCloud2::Request &req,
                                   recon_surface::NormalsFromPointCloud2::Response &res) {
    ROS_INFO("pointNormalsFromPointCloudCallback called");

    try {
        const sensor_msgs::PointCloud2 ptcloud = req.input;
        const geometry_msgs::Point src = req.src;

        time_t rawtime;
        struct tm *timeinfo;
        char buffer[80];

        time(&rawtime);
        timeinfo = localtime(&rawtime);

        strftime(buffer, sizeof(buffer), "%d-%m-%Y_%H:%M:%S", timeinfo);
        std::string str(buffer);

        string output_file = output_folder;
        char endch = output_file.back();
        if ('/' != endch) {
            output_file += "/";
        }
        output_file += buffer;

        ROS_INFO_STREAM("Output file:" << output_file);

        string output_path = generate_point_cloud_normals(ptcloud, src, output_file);

        res.success = true;
        res.path = output_path;
    } catch (std::runtime_error &e) {
        ROS_ERROR("genMeshFromPointCloudCallback exception: %s", e.what());
        res.success = false;
        return false;
    }

    return true;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "p_normal_service_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    ROS_INFO("Start p_normal_service node");

    std::string version(std::to_string(CGAL_VERSION_NR));
    ROS_INFO_STREAM("Using CGAL lib version " << version.substr(1, 2) << "." << version.substr(3, 2));

    // dynamic reconfigure server
    dynamic_reconfigure::Server<recon_surface::ReconSurfaceConfig> server;
    dynamic_reconfigure::Server<recon_surface::ReconSurfaceConfig>::CallbackType f;

    f = boost::bind(&dynReconfigureCallback, _1, _2);
    server.setCallback(f);

    // publishers
    ros::ServiceServer service = nh.advertiseService("/pnormal_from_pointclouds", pointNormalsFromPointCloudCallback);

    ROS_INFO("Spinning p_normal_service node");

    ros::spin();

    ROS_INFO("p_normal_service node stopped");

    return 0;
}

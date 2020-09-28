#include <ros/ros.h>
#include <fstream>

#include "recon_surface/MeshFromPointCloud2.h"

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

ros::Publisher marker_pub_normal;
ros::Publisher marker_pub_inverted;


void publishSTLMarker(ros::Publisher marker_pub, string stl_filepath, string frame_id)
{
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = "file:// " + stl_filepath;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.5;

    marker.lifetime = ros::Duration();
    marker.mesh_use_embedded_materials = true;

    marker_pub.publish(marker);
    ROS_INFO_STREAM("Publishing frame_id: " << frame_id << "stl file:" << stl_filepath);
}

void writeSTLfromMesh(Mesh mesh_obj, string output_stl_filepath, bool flip_normal, bool is_binary_format){
    /*
     * Binary STL format
     * https://github.com/CGAL/cgal/blob/master/Polyhedron_IO/include/CGAL/IO/STL_writer.h
     */

    ofstream out(output_stl_filepath);

    if(is_binary_format) {
        // is binary
        string header = "FileType: Binary                                                                ";
        out.write(header.c_str(), 80);

        const boost::uint32_t N32 = static_cast<boost::uint32_t>(faces(mesh_obj).size());
        out.write(reinterpret_cast<const char *>(&N32), sizeof(N32));

        BOOST_FOREACH(Mesh::Face_index
                              face_index, mesh_obj.faces()) {
                        Mesh::Halfedge_index he = mesh_obj.halfedge(face_index);

                        // Get the x y z position of the 3 verfices of every face
                        vertex_descriptor v0 = mesh_obj.target(he);
                        vertex_descriptor v1 = mesh_obj.target(mesh_obj.next(he));
                        vertex_descriptor v2 = mesh_obj.source(he);

                        CPoint3 p = mesh_obj.point(v0);
                        CPoint3 q = mesh_obj.point(v1);
                        CPoint3 r = mesh_obj.point(v2);

                        if (flip_normal) {
                            q = mesh_obj.point(v0);
                            p = mesh_obj.point(v1);
                        }

                        CGAL::Vector_3<CGAL::Simple_cartesian<double>> n = CGAL::Vector_3<CGAL::Simple_cartesian<double>>(
                                1, 0, 0);
                        if (!CGAL::collinear(p, q, r)) {
                            n = CGAL::unit_normal(p, q, r);
                        }

                        const float coords[12] = {
                                static_cast<float>(n.x()), static_cast<float>(n.y()), static_cast<float>(n.z()),
                                static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()),
                                static_cast<float>(q.x()), static_cast<float>(q.y()), static_cast<float>(q.z()),
                                static_cast<float>(r.x()), static_cast<float>(r.y()), static_cast<float>(r.z())};

                        for (int i = 0; i < 12; ++i)
                            out.write(reinterpret_cast<const char *>(&coords[i]), sizeof(coords[i]));

                        out << "  ";
                    }
    }
    else{
        // is ASCII
        out << "solid STL generated by MeshLab\n";
        BOOST_FOREACH(Mesh::Face_index
                              face_index, mesh_obj.faces()) {
                        Mesh::Halfedge_index he = mesh_obj.halfedge(face_index);

                        // Get the x y z position of the 3 verfices of every face
                        vertex_descriptor v0 = mesh_obj.target(he);
                        vertex_descriptor v1 = mesh_obj.target(mesh_obj.next(he));
                        vertex_descriptor v2 = mesh_obj.source(he);

                        CPoint3 p = mesh_obj.point(v0);
                        CPoint3 q = mesh_obj.point(v1);
                        CPoint3 r = mesh_obj.point(v2);

                        if (flip_normal) {
                            q = mesh_obj.point(v0);
                            p = mesh_obj.point(v1);
                        }

                        CGAL::Vector_3<CGAL::Simple_cartesian<double>> n = CGAL::Vector_3<CGAL::Simple_cartesian<double>>(
                                1, 0, 0);
                        if(!CGAL::collinear(p, q, r)){
                            n = CGAL::unit_normal(p, q, r);
                        }

                        out << "facet normal " << n << "\nouter loop\n";
                        out << "vertex " << p << "\n";
                        out << "vertex " << q << "\n";
                        out << "vertex " << r << "\n";
                        out << "endloop\nendfacet\n";
                    }
        out << "endsolid vcg\n";
    }
}

string generate_mesh(const sensor_msgs::PointCloud2 msg, string output) {
    std::vector<Point> points;
    std::vector<Vector> normals;
    std::vector<Color> colors;
    std::list<PointVectorPair> estimated_pwn;
    std::vector<Point_3> orig_points;
    std::vector<unsigned int> indices;

    FT average_spacing = 0.;

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
        //ROS_INFO_STREAM("Estimating normals");
        estimate_normals(points, estimated_pwn);
    }
    else{
        //ROS_INFO_STREAM("Registering normals normals");
        estimated_pwn = register_normals(points, grab_normals(orig_points, normals));
    }

    stop = chrono::high_resolution_clock::now();
    ROS_INFO_STREAM("Time to estimate normals: "
                            << float(chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0)
                            << " seconds");

    ROS_INFO_STREAM("Points sizes:" << points.size() << " Estimated PWN size:" << estimated_pwn.size());

    start = chrono::high_resolution_clock::now();
    write_ply_wnormals(output + "_simplified_pcd_normals.ply", estimated_pwn, tree, colors);
    stop = chrono::high_resolution_clock::now();
    ROS_INFO_STREAM("Time to write point cloud .ply file: "
                            << float(chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0)
                            << " seconds");

    start = chrono::high_resolution_clock::now();
    trim_mesh(reconstruct_surface(estimated_pwn, output), tree, trim * (double) average_spacing, output);
    stop = chrono::high_resolution_clock::now();
    ROS_INFO_STREAM("Time trim mesh: "
                            << float(chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0)
                            << " seconds");

    start = chrono::high_resolution_clock::now();
    fill_hole(output + "temp3.off", holemaxsize * (double) average_spacing, output);
    stop = chrono::high_resolution_clock::now();
    ROS_INFO_STREAM("Time fill holes: "
                            << float(chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0)
                            << " seconds");

//    start = chrono::high_resolution_clock::now();
//    texture t(tree, colors, output + "_textured.obj", texturesize);
//    t.build_png();
//    t.save_obj();
//    stop = chrono::high_resolution_clock::now();
//    ROS_INFO_STREAM("Time generate *.obj file: "
//                            << float(chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0)
//                            << " seconds");

//    ROS_INFO_STREAM("Writing STL");

    std::ifstream input_off3(output + "temp3.off");
    string output_stl_filepath = output + "_mesh.stl";
    string output_stl_filepath_inverted = output + "_inverted_mesh.stl";
    Mesh mesh_obj;

    if (!input_off3 || !(input_off3 >> mesh_obj)) {
        ROS_ERROR_STREAM("Not a valid off file (generate_mesh): " << output_stl_filepath);
        return "";
    } else {
        start = chrono::high_resolution_clock::now();

        writeSTLfromMesh(mesh_obj, output_stl_filepath, false, true);
        publishSTLMarker(marker_pub_normal, output_stl_filepath, msg.header.frame_id);

        writeSTLfromMesh(mesh_obj, output_stl_filepath_inverted, true, true);
        publishSTLMarker(marker_pub_inverted, output_stl_filepath_inverted, msg.header.frame_id);

        ROS_INFO_STREAM("STL files written to:" << output_stl_filepath << " and " << output_stl_filepath_inverted);
        stop = chrono::high_resolution_clock::now();
        ROS_INFO_STREAM("Time generate *.stl file: "
                                << float(chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0)
                                << " seconds");
    }

    return output_stl_filepath;
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

bool genMeshFromPointCloudCallback(recon_surface::MeshFromPointCloud2::Request &req,
                                   recon_surface::MeshFromPointCloud2::Response &res) {
    ROS_INFO("genMeshFromPointCloudCallback called");

    try {
        const sensor_msgs::PointCloud2 ptcloud = req.input;

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

        string output_path = generate_mesh(ptcloud, output_file);

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
    ros::init(argc, argv, "recon_surface_pointcloud_service_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    ROS_INFO("Start recon_surface_service node");

    std::string version(std::to_string(CGAL_VERSION_NR));
    ROS_INFO_STREAM("Using CGAL lib version " << version.substr(1, 2) << "." << version.substr(3, 2));

    // dynamic reconfigure server
    dynamic_reconfigure::Server<recon_surface::ReconSurfaceConfig> server;
    dynamic_reconfigure::Server<recon_surface::ReconSurfaceConfig>::CallbackType f;

    f = boost::bind(&dynReconfigureCallback, _1, _2);
    server.setCallback(f);

    // publishers
    ros::ServiceServer service = nh.advertiseService("/mesh_from_pointclouds", genMeshFromPointCloudCallback);
    marker_pub_normal = nh.advertise<visualization_msgs::Marker>("/reconstructed_mesh_marker_normal", 1, true);
    marker_pub_inverted = nh.advertise<visualization_msgs::Marker>("/reconstructed_mesh_marker_inverted", 1, true);

    ROS_INFO("Spinning recon_surface_service node");

    ros::spin();

    ROS_INFO("recon_surface_service node stopped");

    return 0;
}

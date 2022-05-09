#ifndef _PLY_PARSER_
#define _PLY_PARSER_

#ifndef CGAL_EIGEN3_ENABLED
#define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_ply_points.h>

#include <CGAL/hierarchy_simplify_point_set.h>
#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/jet_smooth_point_set.h>
#include <CGAL/config.h>
#include <sensor_msgs/PointCloud2.h>

#include <utility>
#include <vector>
#include <fstream>
#include <string>
#include <random>
#include <ros/ros.h>

// types
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
//typedef CGAL::Simple_cartesian<double> Kernel;

typedef Kernel::FT FT;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;

// Color is red/green/blue array
typedef CGAL::cpp11::array<unsigned char, 3> Color;
// Custom interpreter that reads points, normals and colors and stores
// them in the appropriate container
// Concurrency
#ifdef CGAL_LINKED_WITH_TBB
typedef CGAL::Parallel_tag Concurrency_tag;
#else
typedef CGAL::Sequential_tag Concurrency_tag;
#endif

class Pset {
    std::vector<Point> &points;
    std::vector<Vector> &normals;
    std::vector<Color> &colors;

    bool has_normals;
    double sample_ratio;

public:

    Pset(std::vector<Point> &points, std::vector<Vector> &normals, std::vector<Color> &colors, double sample_ratio)
            : points(points), normals(normals), colors(colors), sample_ratio(sample_ratio) {

        has_normals = false;
    }

    void read_pointCloud2(const sensor_msgs::PointCloud2 msg);

    void read_ply(std::string path);

    void write_ply(std::string path);

    void read_csv(std::string path);

    void sample_points(double rm_percent);

    void sample_points_cgal(double param, int method);

    void smooth_pset(unsigned int k);

    void translade_points_mean();

    void correct_color(double v);

};


#endif
//PSET
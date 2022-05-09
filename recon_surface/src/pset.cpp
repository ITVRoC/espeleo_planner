#include "recon_surface/pset.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// Init and test if input file contains the right properties

void Pset::read_pointCloud2(const sensor_msgs::PointCloud2 msg) {

    //set random engine
    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> dis(0, 1);

    pcl::PointCloud<pcl::PointXYZRGBNormal> pclCloud;
    pcl::fromROSMsg(msg, pclCloud);

    Color c = {{0, 0, 0}};

    for (pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator it = pclCloud.points.begin();
         it != pclCloud.points.end(); it++) {

        if (sample_ratio == 1.0 || dis(gen) <= sample_ratio) {
            points.push_back(Point(it->x, it->y, it->z));

            c[0] = it->r;
            c[1] = it->g;
            c[2] = it->b;
            colors.push_back(c);
        }
    }
}


void Pset::read_ply(std::string path) {
    //FT xc = (FT)0., yc = (FT)0., zc = (FT)0.,
    ROS_DEBUG_STREAM("Reading PLY file: " << path);

    Color c = {{0, 0, 0}};
    std::ifstream myfile(path.c_str());
    double x, y, z, nx, ny, nz, r, g, b, alpha;
    bool has_normals = false;
    bool has_alpha = false;

    std::string aux;
    std::stringstream ss;

    //set random engine
    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> dis(0, 1);

    myfile >> aux;

    while (!myfile.eof() && aux != "end_header") {
        if (aux.find("nx") != std::string::npos)
            has_normals = true;
        else if (aux.find("alpha") != std::string::npos)
            has_alpha = true;

        myfile >> aux;
    }

    if (has_normals)
        ROS_DEBUG_STREAM("PLY has normals.");
    if (has_alpha)
        ROS_DEBUG_STREAM("PLY has alpha channel");

    myfile >> x >> y >> z;
    if (has_normals)
        myfile >> nx >> ny >> nz;
    myfile >> r >> g >> b;
    if (has_alpha)
        myfile >> alpha;

    while (!myfile.eof()) {
        if (sample_ratio == 1.0 || dis(gen) <= sample_ratio) {
            points.push_back(Point(x, y, z));
            c[0] = r;
            c[1] = g;
            c[2] = b;
            colors.push_back(c);

            if (has_normals)
                normals.push_back(Vector(nx, ny, nz));
        }

        myfile >> x >> y >> z;
        if (has_normals)
            myfile >> nx >> ny >> nz;
        myfile >> r >> g >> b;
        if (has_alpha)
            myfile >> alpha;
    }
}


void Pset::read_csv(std::string path) {
    //FT xc = (FT)0., yc = (FT)0., zc = (FT)0.,
    ROS_DEBUG_STREAM("Reading CSV file: " << path);

    Color c = {{255, 255, 255}};
    std::ifstream myfile(path.c_str());
    double x, y, z, r, g, b;
    char v;

    std::string aux;

    //set random engine
    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> dis(0, 1);

    myfile >> aux; //header
    bool has_color = false;
    if (aux.size() > 2 * 3 + 1)
        has_color = true;

    if (has_color) {
        ROS_DEBUG_STREAM("Reading colored points.");

        myfile >> x >> v >> y >> v >> z >> v >> r >> v >> g >> v >> b;

        while (!myfile.eof()) {
            if (sample_ratio == 1.0 || dis(gen) <= sample_ratio) {
                points.push_back(Point(x, y, z));
                c[0] = r;
                c[1] = g;
                c[2] = b;
                colors.push_back(c);
            }

            myfile >> x >> v >> y >> v >> z >> v >> r >> v >> g >> v >> b;
        }
    } else {
        ROS_DEBUG_STREAM("Reading points without color.");
        myfile >> x >> v >> y >> v >> z;

        while (!myfile.eof()) {
            if (sample_ratio == 1.0 || dis(gen) <= sample_ratio) {
                myfile >> x >> v >> y >> v >> z;
                points.push_back(Point(x, y, z));
                colors.push_back(c);
            }

        }
    }
}


void Pset::sample_points(double rm_percent) {
    std::vector<Point> n_points;
    std::vector<Vector> n_normals;
    std::vector<Color> n_colors;

    double skip = 0;

    for (size_t i = 0; i < points.size(); i++) {
        skip += rm_percent;
        if (skip >= 1.0) {
            skip -= 1;

            n_points.push_back(points[i]);

            if (normals.size() > 0)
                n_normals.push_back(normals[i]);

            n_colors.push_back(colors[i]);
        }
    }

    points = n_points;
    normals = n_normals;
    colors = n_colors;
}

void Pset::sample_points_cgal(double param, int method) {
    std::vector<Point> output;

    ROS_DEBUG_STREAM("Sample points:" << param);

    if (method == 0) {
        int max_cluster_size = (int) param;
        // points.erase(
        //         CGAL::hierarchy_simplify_point_set(
        //                 points.begin(),
        //                 points.end(),
        //                 max_cluster_size, // Max cluster size
        //                 0.02), // Max surface variation
        //         points.end());

    } else if (method == 1) {
        double cell_size = param;
        points.erase(
                CGAL::grid_simplify_point_set(
                        points.begin(),
                        points.end(),
                        cell_size),

                points.end());

        // Optional: after erase(), use Scott Meyer's "swap trick" to trim excess capacity
        std::vector<Point>(points).swap(points);
    }
}

void Pset::smooth_pset(unsigned int k) {
    CGAL::jet_smooth_point_set<Concurrency_tag>(points.begin(), points.end(), k);
}

void Pset::translade_points_mean() {

    double mx, my, mz;
    mx = my = mz = 0;

    //for(unsigned int i=0;i<points.size();i++)
    //  std::cout<<std::setprecision(36)<<points[i]<<std::endl;

    std::vector<Point> shift_points;

    for (unsigned int i = 0; i < points.size(); i++) {
        mx += points[i][0];
        my += points[i][1];
        mz += points[i][2];
    }

    mx /= (double) points.size();
    my /= (double) points.size();
    mz /= (double) points.size();

    for (unsigned int i = 0; i < points.size(); i++)
        shift_points.push_back(Point(points[i][0] - mx, points[i][1] - my, points[i][2] - mz));

    points = shift_points;
}

void Pset::write_ply(std::string path) {
    static char ply_header[] =
            "ply\n"
            "format ascii 1.0\n"
            "element face 0\n"
            "property list uchar int vertex_indices\n"
            "element vertex %ld\n"
            "property float x\n"
            "property float y\n"
            "property float z\n"
            "property uchar red\n"
            "property uchar green\n"
            "property uchar blue\n"
            "end_header\n";

    size_t num_points_out = points.size();
    FILE *f = fopen(path.c_str(), "w");
    /* Print the ply header */
    fprintf(f, ply_header, num_points_out);

    /* X Y Z R G B for each line*/
    for (size_t i = 0; i < num_points_out; i++)
        fprintf(f, "%.7f %.7f %.7f %d %d %d\n", points[i][0], points[i][1], points[i][2], colors[i][0], colors[i][1],
                colors[i][2]);

    fclose(f);
}

void Pset::correct_color(double val) {
    for (size_t i = 0; i < colors.size(); i++) {
        double l = (colors[i][0] + colors[i][1] + colors[i][2]) / (3 * 255.0) + 1e-3;

        if (l < val) {
            double cval = val / l;

            unsigned char r = ((unsigned char) (cval * colors[i][0]) <= 255) ? (unsigned char) (cval * colors[i][0])
                                                                             : 255;
            unsigned char g = ((unsigned char) (cval * colors[i][1]) <= 255) ? (unsigned char) (cval * colors[i][1])
                                                                             : 255;
            unsigned char b = ((unsigned char) (cval * colors[i][2]) <= 255) ? (unsigned char) (cval * colors[i][2])
                                                                             : 255;
            colors[i][0] = r;
            colors[i][1] = g;
            colors[i][2] = b;
        }
    }
}
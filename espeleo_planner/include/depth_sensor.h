#include <octomap/octomap.h>
#include <ros/console.h>
#include <octomap/math/Utils.h>
#include <octomap/math/Vector3.h>

struct DepthSensor {
    double horizontal_fov;
    double vertical_fov;
    double angle_inc_hor;
    double angle_inc_vel;
    double width;
    double height;
    double max_range;

    octomap::Pointcloud sensor_rays;
    octomap::point3d initial_vector;

    DepthSensor(double _width, double _height, double _horizontal_fov, double _vertical_fov, double _max_range)
            : width(_width), height(_height), horizontal_fov(_horizontal_fov), vertical_fov(_vertical_fov), max_range(_max_range) {

        angle_inc_hor = horizontal_fov / width;
        angle_inc_vel = vertical_fov / height;

        for(double j = -height / 2; j < height / 2; ++j) {
            for (double i = -width / 2; i < width / 2; ++i) {
                initial_vector = octomap::point3d(1.0, 0.0, 0.0);
                initial_vector.rotate_IP(0.0, j * angle_inc_vel, i * angle_inc_hor);
                sensor_rays.push_back(initial_vector);
            }
        }
    }
};
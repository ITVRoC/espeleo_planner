#include <iostream>
#include <string>
#include <Eigen/Dense>
#include "surface_recon.hpp"
#include <ros/ros.h>

bool check_pca_threshold(std::vector<Point_3> pts, float threshold,
                         int max); //compares ratio between largest and smallest eigenvalue
void rm_artifacts_mesh(std::string path, int k);

CPoint3 laplacian_smooth(std::vector<Point_3> pts);
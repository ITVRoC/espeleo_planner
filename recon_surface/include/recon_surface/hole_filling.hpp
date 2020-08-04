#ifndef _HOLE_FILLING_
#define _HOLE_FILLING_

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <boost/foreach.hpp>
#include <ros/ros.h>


typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Polyhedron_3<K> P;
typedef P::Halfedge_handle H;
typedef P::Facet_handle F;
typedef P::Vertex_handle V;
typedef P::Point_3  Point3;


int fill_hole(std::string mesh_path, double max_perimeter);

#endif 
// HOLE_FILLING

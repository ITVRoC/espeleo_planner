#ifndef _SURFACE_RECON_
#define _SURFACE_RECON_

#ifndef CGAL_EIGEN3_ENABLED
  #define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_xyz_points.h>
#include <utility> // defines std::pair
#include <list>
#include <fstream>
#include <stdio.h>

#include <CGAL/trace.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/Surface_mesh_default_triangulation_3.h>
#include <CGAL/make_surface_mesh.h>
#include <CGAL/Implicit_surface_3.h>
#include <CGAL/IO/output_surface_facets_to_polyhedron.h>
#include <CGAL/Poisson_reconstruction_function.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/compute_average_spacing.h>
#include <vector>


//Search tree

#include <CGAL/Search_traits_3.h>
#include <CGAL/Search_traits_adapter.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <boost/iterator/zip_iterator.hpp>
#include <utility>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

#include <CGAL/Polygon_mesh_processing/connected_components.h>

typedef CGAL::Simple_cartesian<double>::Point_3 CPoint3;
typedef CGAL::Surface_mesh<CPoint3> Mesh;
typedef Mesh::Vertex_index vertex_descriptor;

// Types
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::FT FT;
typedef Kernel::Point_3 Point;
typedef CGAL::Point_with_normal_3<Kernel> Point_with_normal;
typedef Kernel::Sphere_3 Sphere;
typedef std::vector<Point_with_normal> PointList;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef CGAL::Poisson_reconstruction_function<Kernel> Poisson_reconstruction_function;
typedef CGAL::Surface_mesh_default_triangulation_3 STr;
typedef CGAL::Surface_mesh_complex_2_in_triangulation_3<STr> C2t3;
typedef CGAL::Implicit_surface_3<Kernel, Poisson_reconstruction_function> Surface_3;

// Types

typedef Kernel::Vector_3 Vector;
typedef CGAL::cpp11::array<unsigned char, 3> Color;
// Point with normal vector stored in a std::pair.
typedef std::pair<Point, Vector> PointVectorPair;
// Concurrency
#ifdef CGAL_LINKED_WITH_TBB
typedef CGAL::Parallel_tag Concurrency_tag;
#else
typedef CGAL::Sequential_tag Concurrency_tag;
#endif

typedef Kernel::Point_3                                     Point_3;
typedef boost::tuple<Point_3,int>                           Point_and_int;
typedef CGAL::Search_traits_3<Kernel>                       Traits_base;
typedef CGAL::Search_traits_adapter<Point_and_int,
  CGAL::Nth_of_tuple_property_map<0, Point_and_int>,
  Traits_base>                                              Traits;
typedef CGAL::Orthogonal_k_neighbor_search<Traits>          K_neighbor_search;
typedef K_neighbor_search::Tree                             Tree;
typedef K_neighbor_search::Distance                         Distance;

void translade_pts_mean(std::vector<Point> &pts);
std::list<PointVectorPair> grab_normals(std::vector<Point>& pts, std::vector<Vector>& norms);
void estimate_normals(std::vector<Point>& pts,std::list<PointVectorPair>& points);
std::list<PointVectorPair> register_normals(std::vector<Point> sampled_points, std::list<PointVectorPair> original);
Mesh reconstruct_surface(std::list <PointVectorPair>& pwn);
void write_ply_wnormals(std::string out, std::list<PointVectorPair>& point_list, Tree& tree, std::vector<Color>& colors);
void trim_mesh(Mesh m, Tree& tree, double average_spacing);



#endif 
//_SURFACE_RECON_

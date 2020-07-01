#include "surface_recon.hpp"


std::list<PointVectorPair> grab_normals(std::vector<Point>& pts, std::vector<Vector>& norms)
{
     std::list<PointVectorPair> points;

   for(unsigned int i=0; i< pts.size();i++)
      points.push_back(std::make_pair(pts[i],norms[i]));

    std::cout<<"Using provided normals..."<<std::endl;

    return points;

}

void estimate_normals(std::vector<Point>& pts, std::list<PointVectorPair>& points )
{
    points.clear();

	 for(unsigned int i=0; i< pts.size();i++)
	 	points.push_back(std::make_pair(pts[i],Vector((FT)0.,(FT)0.,(FT)0.)));

	 // Estimates normals direction.
    // Note: pca_estimate_normals() requires an iterator over points
    // as well as property maps to access each point's position and normal.
    const int nb_neighbors = 18;//18; // K-nearest neighbors = 3 rings
    CGAL::pca_estimate_normals<Concurrency_tag>(points.begin(), points.end(),
                               CGAL::First_of_pair_property_map<PointVectorPair>(),
                               CGAL::Second_of_pair_property_map<PointVectorPair>(),
                               nb_neighbors);
    // Orients normals.
    // Note: mst_orient_normals() requires an iterator over points
    // as well as property maps to access each point's position and normal.
    std::list<PointVectorPair>::iterator unoriented_points_begin =
      CGAL::mst_orient_normals(points.begin(), points.end(),
                                 CGAL::First_of_pair_property_map<PointVectorPair>(),
                                 CGAL::Second_of_pair_property_map<PointVectorPair>(),
                                 nb_neighbors + 48);
    // Optional: delete points with an unoriented normal
    // if you plan to call a reconstruction algorithm that expects oriented normals.
    points.erase(unoriented_points_begin, points.end());
  

    #ifdef DEBUG_TRUE

    for(std::list<PointVectorPair>::iterator it = points.begin(); it != points.end();++it)
    	std::cout<<std::setprecision(12)<<"x y z: "<<(*it).first<<"  nx ny nz: "<<(*it).second<<std::endl;

    #endif
    std::cout<<"normals estimated."<<std::endl;

    for(std::list<PointVectorPair>::iterator it = points.begin(); it != points.end();++it)
    {
    	(*it).second = Vector((*it).second[0],(*it).second[1],(*it).second[2]);
    }
  
}

std::list<PointVectorPair> register_normals(std::vector<Point> sampled_points, std::list<PointVectorPair> original)
{
  std::vector<Point_3> orig_points;
  std::vector<Vector> orig_normals;
  std::vector<unsigned int> indices;
  std::list<PointVectorPair> refined;

  size_t tl=0;

   for(std::list<PointVectorPair>::iterator it = original.begin(); it != original.end();++it,tl++) //initialize structure for Kdtree
  {
    orig_points.push_back(it->first);
    orig_normals.push_back(it->second);
    indices.push_back(tl);
  }

  // Initialize a spatial kd-tree for further search on original set of points
  Tree tree(
    boost::make_zip_iterator(boost::make_tuple( orig_points.begin(),indices.begin() )),
    boost::make_zip_iterator(boost::make_tuple( orig_points.end(),indices.end() ) )  
  );

//find nearest neighbor and grab the normals

  for(size_t i=0; i<sampled_points.size();i++)
  {

     K_neighbor_search search(tree, sampled_points[i], 1);
     unsigned int idx;

     for(K_neighbor_search::iterator it = search.begin(); it != search.end(); it++)
       idx = boost::get<1>(it->first);

     refined.push_back(std::make_pair(sampled_points[i],orig_normals[idx]));

  }

  std::cout<<"Normals registered and refined..."<<std::endl;
  return refined;

}

Mesh reconstruct_surface(std::list <PointVectorPair>& pwn)
{
	    // Poisson options
    FT sm_angle = 20.0; //20.0 Min triangle angle in degrees.
    FT sm_radius = 8.0; // Max triangle size w.r.t. point set average spacing. //10.0
    FT sm_distance = 0.4;//0.375; // Surface Approximation error w.r.t. point set average spacing. //0.5
    // Reads the point set file in points[].
    // Note: read_xyz_points_and_normals() requires an iterator over points
    // + property maps to access each point's position and normal.
    // The position property map can be omitted here as we use iterators over Point_3 elements.
    PointList points;
    
    //Converts pwn to points structure
    points.resize(pwn.size());
    unsigned int idx = 0;

    for(std::list<PointVectorPair>::iterator it = pwn.begin(); it != pwn.end();++it,idx++)
    {
      	points[idx].position() = (*it).first;
        points[idx].normal() = (*it).second;
    }

    std::cout<<"Surface recon started..."<<std::endl;

    // Creates implicit function from the read points using the default solver.
    // Note: this method requires an iterator over points
    // + property maps to access each point's position and normal.
    // The position property map can be omitted here as we use iterators over Point_3 elements.
    Poisson_reconstruction_function function(points.begin(), points.end(),
                                             CGAL::make_normal_of_point_with_normal_pmap(PointList::value_type()) );
    // Computes the Poisson indicator function f()
    // at each vertex of the triangulation.

    
    if ( ! function.compute_implicit_function() ) 
      std::cout<<"Error in computing implicit function"<<std::endl;
    // Computes average spacing

  	std::cout<<"Implicit function computed..."<<std::endl;
    FT average_spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>(points.begin(), points.end(),
                                                       6 /* knn = 1 ring */);
    // Gets one point inside the implicit surface
    // and computes implicit function bounding sphere radius.
    Point inner_point = function.get_inner_point();
    Sphere bsphere = function.bounding_sphere();
    FT radius = std::sqrt(bsphere.squared_radius());
    // Defines the implicit surface: requires defining a
    // conservative bounding sphere centered at inner point.
    FT sm_sphere_radius = 5.0 * radius;
    FT sm_dichotomy_error = sm_distance*average_spacing/1000.0; // Dichotomy error must be << sm_distance
    Surface_3 surface(function,
                      Sphere(inner_point,sm_sphere_radius*sm_sphere_radius),
                      sm_dichotomy_error/sm_sphere_radius);
    // Defines surface mesh generation criteria
    CGAL::Surface_mesh_default_criteria_3<STr> criteria(sm_angle,  // Min triangle angle (degrees)
                                                        sm_radius*average_spacing,  // Max triangle size
                                                        sm_distance*average_spacing); // Approximation error
    // Generates surface mesh with manifold option
    STr tr; // 3D Delaunay triangulation for surface mesh generation
    C2t3 c2t3(tr); // 2D complex in 3D Delaunay triangulation
    std::cout<<"3D Delaunay triangulation..."<<std::endl;
    CGAL::make_surface_mesh(c2t3,                                 // reconstructed mesh
                            surface,                              // implicit surface
                            criteria,                             // meshing criteria
                            CGAL::Manifold_with_boundary_tag());  // require manifold mesh
    if(tr.number_of_vertices() == 0)
      std::cout<<"Warning: mesh has 0 vertices"<<std::endl;
    // saves reconstructed surface mesh
    std::ofstream out("temp1.off");
    std::ifstream in("temp1.off");
    Polyhedron output_mesh;
    CGAL::output_surface_facets_to_polyhedron(c2t3, output_mesh);

    out << output_mesh;

    Mesh m;

    in >> m;

    return m;
}

void write_ply_wnormals(std::string out_file, std::list<PointVectorPair>& point_list, Tree& tree, std::vector<Color>& colors)
{
  /*Point_3 query(0.0, 0.0, 0.0);
  
  // search K nearest neighbours
  K_neighbor_search search(tree, query, 1);
  for(K_neighbor_search::iterator it = search.begin(); it != search.end(); it++)
  {
    std::cout << " d(q, nearest neighbor)=  "
          << tr_dist.inverse_of_transformed_distance(it->second) << " " 
              << boost::get<0>(it->first)<< " " << boost::get<1>(it->first) << std::endl;
    

    unsigned int idx =  boost::get<1>(it->first);
  }*/

    Distance tr_dist;

    static char ply_header[] =
   "ply\n"
   "format ascii 1.0\n"
   "element face 0\n"
   "property list uchar int vertex_indices\n"
   "element vertex %ld\n"
   "property float x\n"
   "property float y\n"
   "property float z\n"
   "property float nx\n"
   "property float ny\n"
   "property float nz\n"
   "property uchar red\n"
   "property uchar green\n"
   "property uchar blue\n"
   "end_header\n";

   long num_points_out = point_list.size();

   FILE *f = fopen(out_file.c_str(), "w");

       /* Print the ply header */
       fprintf(f, ply_header, num_points_out);

       /* X Y Z R G B for each line*/

       for(std::list<PointVectorPair>::iterator it = point_list.begin(); it != point_list.end(); ++it)
       {
           Point pt3d = (*it).first;
           Vector n3d = (*it).second;
           
           //find nearest neighbor and grab the color
            K_neighbor_search search(tree, pt3d, 1);
            unsigned int idx;

            for(K_neighbor_search::iterator it = search.begin(); it != search.end(); it++)
              idx = boost::get<1>(it->first);

           Color cl = colors[idx];

           fprintf(f,"%.7f %.7f %.7f %.7f %.7f %.7f %d %d %d\n",pt3d[0],pt3d[1],pt3d[2],n3d[0],n3d[1],n3d[2],cl[0],cl[1],cl[2]);

       }
       fclose(f);
}

void trim_mesh(Mesh m, Tree& tree, double threshold)
{
   Distance tr_dist;
  std::cout<<"threshold for trim: "<<threshold<<std::endl;


   {

    BOOST_FOREACH(vertex_descriptor vd, m.vertices())
    {
          //std::cout << m.point(vd) << std::endl;
         K_neighbor_search search(tree, Point(m.point(vd)[0],m.point(vd)[1],m.point(vd)[2]), 1);
         double distance;

        for(K_neighbor_search::iterator it = search.begin(); it != search.end(); it++)
          distance =  tr_dist.inverse_of_transformed_distance(it->second);

        if(distance > threshold)
        {
          m.remove_vertex(vd);
        }
     
     }

      BOOST_FOREACH(Mesh::Face_index face_index, m.faces())
    {
      Mesh::Halfedge_index he = m.halfedge(face_index);
      vertex_descriptor v0 = m.target(he);
      vertex_descriptor v1 = m.target(m.next(he));
      vertex_descriptor v2 = m.target(m.prev(he));

      if (m.is_removed(v0) || m.is_removed(v1) || m.is_removed(v2))
        m.remove_face(face_index);
    }

   }
     

   //std::cout<<"Removing disconnected components..."<<std::endl;
   //CGAL::Polygon_mesh_processing::keep_largest_connected_components(m,1);
   //std::cout<<"Done."<<std::endl;
   m.collect_garbage();
   std::ofstream out("temp2.off");
   out << m;

}
#include "recon_surface/surface_recon.hpp"


std::list<PointVectorPair> grab_normals(std::vector<Point> &pts, std::vector<Vector> &norms) {
    std::list<PointVectorPair> points;

    for (unsigned int i = 0; i < pts.size(); i++)
        points.push_back(std::make_pair(pts[i], norms[i]));

    ROS_DEBUG_STREAM("Using provided normals...");

    return points;

}

void estimate_normals(std::vector<Point> &pts, std::list<PointVectorPair> &points) {
    points.clear();

    // std::sort(pts.begin(), pts.end(), ComparePointsToOrigin());

    for (unsigned int i = 0; i < pts.size(); i++)
        points.push_back(std::make_pair(pts[i], Vector((FT) 0., (FT) 0., (FT) 0.)));

//    for(std::list<PointVectorPair>::iterator it = points.begin(); it != points.end();++it)
//        std::cout<<std::setprecision(12)<<"x y z: "<<(*it).first<<"  nx ny nz: "<<(*it).second<<std::endl;

    ROS_DEBUG_STREAM("Num points:" << points.size());

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
    //    std::list<PointVectorPair>::iterator unoriented_points_begin =
    //            CGAL::mst_orient_normals(points.begin(), points.end(),
    //                                     CGAL::First_of_pair_property_map<PointVectorPair>(),
    //                                     CGAL::Second_of_pair_property_map<PointVectorPair>(),
    //                                     nb_neighbors + 48);

    std::list<PointVectorPair>::iterator unoriented_points_begin =
            mst_orient_normals_modified(points.begin(), points.end(),
                                     CGAL::First_of_pair_property_map<PointVectorPair>(),
                                     CGAL::Second_of_pair_property_map<PointVectorPair>(),
                                     nb_neighbors + 48,
                                     Kernel());

    // Optional: delete points with an unoriented normal
    // if you plan to call a reconstruction algorithm that expects oriented normals.
    points.erase(unoriented_points_begin, points.end());


#ifdef DEBUG_TRUE
    for(std::list<PointVectorPair>::iterator it = points.begin(); it != points.end();++it)
        std::cout<<std::setprecision(12)<<"x y z: "<<(*it).first<<"  nx ny nz: "<<(*it).second<<std::endl;
#endif

    ROS_DEBUG_STREAM("Normals estimated.");

    for (std::list<PointVectorPair>::iterator it = points.begin(); it != points.end(); ++it) {
        (*it).second = Vector((*it).second[0], (*it).second[1], (*it).second[2]);
    }
}

template <typename ForwardIterator,
        typename PointPMap,
        typename NormalPMap,
        typename Kernel
>
ForwardIterator
mst_orient_normals_modified(
        ForwardIterator first,  ///< iterator over the first input point.
        ForwardIterator beyond, ///< past-the-end iterator over the input points.
        PointPMap point_pmap, ///< property map: value_type of ForwardIterator -> Point_3.
        NormalPMap normal_pmap, ///< property map: value_type of ForwardIterator -> Vector_3.
        unsigned int k, ///< number of neighbors
        const Kernel& kernel) ///< geometric traits.
{
    ROS_DEBUG_STREAM("Calls mst_orient_normals()\n");

    // Input points types
    typedef typename std::iterator_traits<ForwardIterator>::value_type Enriched_point; // actual type of input points
    // Property map ForwardIterator -> index
    typedef CGAL::Index_property_map<ForwardIterator> IndexPMap;

    // Riemannian_graph types
    typedef CGAL::internal::Riemannian_graph<ForwardIterator> Riemannian_graph;

    // MST_graph types
    typedef CGAL::internal::MST_graph<ForwardIterator, NormalPMap, Kernel> MST_graph;

    // Precondition: at least one element in the container.
    CGAL_point_set_processing_precondition(first != beyond);

    // Precondition: at least 2 nearest neighbors
    CGAL_point_set_processing_precondition(k >= 2);

    std::size_t memory = CGAL::Memory_sizer().virtual_size(); CGAL_TRACE("  %ld Mb allocated\n", memory>>20);
    ROS_DEBUG_STREAM("  Create Index_property_map\n");

    // Create a property map Iterator -> index.
    // - if ForwardIterator is a random access iterator (typically vector and deque),
    // get() just calls std::distance() and is very efficient;
    // - else, the property map allocates a std::map to store indices
    // and get() requires a lookup in the map.
    IndexPMap index_pmap(first, beyond);

    // Orients the normal of the point with maximum Z towards +Z axis.
    ForwardIterator first2 = first;
    ForwardIterator source_point
            = mst_find_source(first, beyond,
                              point_pmap, normal_pmap,
                              kernel);

    // Find the closer point to the 0, 0, 0
    // This is a heuristic to prevent the reconstruction of only
    // small remote poitn cloud components
    // This point will be used as origin by the MST algorithm over the
    // Riemannian Graph
    ForwardIterator closest_point = first2;
    double p_dist = std::sqrt(
            std::pow(get(point_pmap,*closest_point).x(), 2) +
            std::pow(get(point_pmap,*closest_point).y(), 2) +
            std::pow(get(point_pmap,*closest_point).z(), 2));

    double local_dist = 0;
    for (ForwardIterator v = ++first2; v != beyond; v++)
    {
        local_dist = std::sqrt(
                std::pow(get(point_pmap,*v).x(), 2) +
                std::pow(get(point_pmap,*v).y(), 2) +
                std::pow(get(point_pmap,*v).z(), 2));

        if (local_dist < p_dist){
            closest_point = v;
            p_dist = local_dist;
//            ROS_DEBUG_STREAM("New source: x:" << get(point_pmap,*v).x() << " y:" << get(point_pmap,*v).y() <<
//            " z:" << get(point_pmap,*v).z() << " dist:" << local_dist);
        }
    }

/*
 * ForwardIterator source_point = first;
    typedef typename boost::property_traits<NormalPMap>::value_type Vector;
    typedef typename boost::property_traits<NormalPMap>::reference Vector_ref;
    // Orients its normal towards +Z axis
    Vector_ref normal = get(normal_pmap,*source_point);
    const Vector Z(0, 0, 1);
    if (Z * normal < 0) {
        CGAL_TRACE("  Flip top point normal\n");
        put(normal_pmap,*source_point, -normal);
    }
*/

    // Iterates over input points and creates Riemannian Graph:
    // - vertices are numbered like the input points index.
    // - vertices are empty.
    // - we add the edge (i, j) if either vertex i is in the k-neighborhood of vertex j,
    //   or vertex j is in the k-neighborhood of vertex i.
    Riemannian_graph riemannian_graph
            = create_riemannian_graph(first, beyond,
                                      point_pmap, normal_pmap, index_pmap,
                                      k,
                                      kernel);

    const std::size_t num_input_points = num_vertices(riemannian_graph);
    ROS_DEBUG_STREAM("Num vertices riemannian graph:" << num_input_points);

    // Creates a Minimum Spanning Tree starting at source_point
    MST_graph mst_graph = create_mst_graph(first, beyond,
                                           point_pmap, normal_pmap, index_pmap,
                                           k,
                                           kernel,
                                           riemannian_graph,
                                           closest_point); // originally this was source_point

    const std::size_t num_input_points2 = num_vertices(mst_graph);
    ROS_DEBUG_STREAM("Num vertices mst graph:" << num_input_points2);

    memory = CGAL::Memory_sizer().virtual_size();
    double mb_memory = memory >> 20;
    ROS_DEBUG_STREAM(mb_memory << "Mb allocated");
    ROS_DEBUG_STREAM("  Calls boost::breadth_first_search()");

    // Traverse the point set along the MST to propagate source_point's orientation
    CGAL::internal::Propagate_normal_orientation<ForwardIterator, NormalPMap, Kernel> orienter;
    std::size_t source_point_index = get(index_pmap, closest_point);
    boost::breadth_first_search(mst_graph,
                                vertex(source_point_index, mst_graph), // source
                                visitor(boost::make_bfs_visitor(orienter)));

    // Copy points with robust normal orientation to oriented_points[], the others to unoriented_points[].
    std::deque<Enriched_point> oriented_points, unoriented_points;
    for (ForwardIterator it = first; it != beyond; it++)
    {
        std::size_t it_index = get(index_pmap,it);
        typename MST_graph::vertex_descriptor v = vertex(it_index, mst_graph);
        if (mst_graph[v].is_oriented)
            oriented_points.push_back(*it);
        else
            unoriented_points.push_back(*it);
    }

    // Replaces [first, beyond) range by the content of oriented_points[], then unoriented_points[].
    ForwardIterator first_unoriented_point =
            std::copy(oriented_points.begin(), oriented_points.end(), first);
    std::copy(unoriented_points.begin(), unoriented_points.end(), first_unoriented_point);

    // At this stage, we have typically 0 unoriented normals if k is large enough
    CGAL_TRACE("  => %u normals are unoriented\n", unoriented_points.size());

    memory = CGAL::Memory_sizer().virtual_size(); CGAL_TRACE("  %ld Mb allocated\n", memory>>20);
    CGAL_TRACE("End of mst_orient_normals()\n");

    return first_unoriented_point;
}

std::list<PointVectorPair> register_normals(std::vector<Point> sampled_points, std::list<PointVectorPair> original) {
    std::vector<Point_3> orig_points;
    std::vector<Vector> orig_normals;
    std::vector<unsigned int> indices;
    std::list<PointVectorPair> refined;

    size_t tl = 0;

    for (std::list<PointVectorPair>::iterator it = original.begin();
         it != original.end(); ++it, tl++) //initialize structure for Kdtree
    {
        orig_points.push_back(it->first);
        orig_normals.push_back(it->second);
        indices.push_back(tl);
    }

    // Initialize a spatial kd-tree for further search on original set of points
    Tree tree(
            boost::make_zip_iterator(boost::make_tuple(orig_points.begin(), indices.begin())),
            boost::make_zip_iterator(boost::make_tuple(orig_points.end(), indices.end()))
    );

    //find nearest neighbor and grab the normals
    for (size_t i = 0; i < sampled_points.size(); i++) {

        K_neighbor_search search(tree, sampled_points[i], 1);
        unsigned int idx;

        for (K_neighbor_search::iterator it = search.begin(); it != search.end(); it++)
            idx = boost::get<1>(it->first);

        refined.push_back(std::make_pair(sampled_points[i], orig_normals[idx]));
    }

    ROS_DEBUG_STREAM("Normals registered and refined...");
    return refined;
}

Mesh reconstruct_surface(std::list<PointVectorPair> &pwn, std::string base_path) {
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

    for (std::list<PointVectorPair>::iterator it = pwn.begin(); it != pwn.end(); ++it, idx++) {
        points[idx].position() = (*it).first;
        points[idx].normal() = (*it).second;
    }

    ROS_DEBUG_STREAM("Surface recon started...");

    // Creates implicit function from the read points using the default solver.
    // Note: this method requires an iterator over points
    // + property maps to access each point's position and normal.
    // The position property map can be omitted here as we use iterators over Point_3 elements.
    Poisson_reconstruction_function function(points.begin(), points.end(),
                                             CGAL::make_normal_of_point_with_normal_pmap(PointList::value_type()));
    // Computes the Poisson indicator function f()
    // at each vertex of the triangulation.


    if (!function.compute_implicit_function())
        ROS_DEBUG_STREAM("Error in computing implicit function");
    // Computes average spacing

    ROS_DEBUG_STREAM("Implicit function computed...");

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
    FT sm_dichotomy_error = sm_distance * average_spacing / 1000.0; // Dichotomy error must be << sm_distance
    Surface_3 surface(function,
                      Sphere(inner_point, sm_sphere_radius * sm_sphere_radius),
                      sm_dichotomy_error / sm_sphere_radius);
    // Defines surface mesh generation criteria
    CGAL::Surface_mesh_default_criteria_3<STr> criteria(sm_angle,  // Min triangle angle (degrees)
                                                        sm_radius * average_spacing,  // Max triangle size
                                                        sm_distance * average_spacing); // Approximation error
    // Generates surface mesh with manifold option
    STr tr; // 3D Delaunay triangulation for surface mesh generation
    C2t3 c2t3(tr); // 2D complex in 3D Delaunay triangulation

    ROS_DEBUG_STREAM("3D Delaunay triangulation...");

    CGAL::make_surface_mesh(c2t3,                                 // reconstructed mesh
                            surface,                              // implicit surface
                            criteria,                             // meshing criteria
                            CGAL::Manifold_with_boundary_tag());  // require manifold mesh
    if (tr.number_of_vertices() == 0)
        ROS_DEBUG_STREAM("Warning: mesh has 0 vertices");

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

void write_ply_wnormals(std::string out_file, std::list<PointVectorPair> &point_list, Tree &tree,
                        std::vector<Color> &colors) {
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
    for (std::list<PointVectorPair>::iterator it = point_list.begin(); it != point_list.end(); ++it) {
        Point pt3d = (*it).first;
        Vector n3d = (*it).second;

        //find nearest neighbor and grab the color
        K_neighbor_search search(tree, pt3d, 1);
        unsigned int idx;

        for (K_neighbor_search::iterator it = search.begin(); it != search.end(); it++)
            idx = boost::get<1>(it->first);

        Color cl = colors[idx];

        fprintf(f, "%.7f %.7f %.7f %.7f %.7f %.7f %d %d %d\n", pt3d[0], pt3d[1], pt3d[2], n3d[0], n3d[1], n3d[2], cl[0],
                cl[1], cl[2]);

    }
    fclose(f);
}

void trim_mesh(Mesh m, Tree &tree, double threshold, std::string base_path) {
    Distance tr_dist;
    ROS_DEBUG_STREAM("Threshold for trim: " << threshold);

    BOOST_FOREACH(vertex_descriptor vd, m.vertices()) {
                    //std::cout << m.point(vd) << std::endl;
                    K_neighbor_search search(tree, Point(m.point(vd)[0], m.point(vd)[1], m.point(vd)[2]), 1);
                    double distance;

                    for (K_neighbor_search::iterator it = search.begin(); it != search.end(); it++)
                        distance = tr_dist.inverse_of_transformed_distance(it->second);

                    if (distance > threshold) {
                        m.remove_vertex(vd);
                    }

                }

    BOOST_FOREACH(Mesh::Face_index
                          face_index, m.faces()) {
                    Mesh::Halfedge_index he = m.halfedge(face_index);
                    vertex_descriptor v0 = m.target(he);
                    vertex_descriptor v1 = m.target(m.next(he));
                    vertex_descriptor v2 = m.target(m.prev(he));

                    if (m.is_removed(v0) || m.is_removed(v1) || m.is_removed(v2))
                        m.remove_face(face_index);
                }

    //std::cout<<"Removing disconnected components..."<<std::endl;
    //CGAL::Polygon_mesh_processing::keep_largest_connected_components(m,1);
    //std::cout<<"Done."<<std::endl;
    m.collect_garbage();
    std::ofstream out(base_path + "temp2.off");
    out << m;
}
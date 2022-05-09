#include "recon_surface/hole_filling.hpp"

int fill_hole(std::string mesh_path, double max_perimeter, std::string base_path) {

    std::string filename = base_path + "temp2.off";
    //const char *filename = base_temp;
    std::ifstream input(filename);
    P poly;
    if (!input || !(input >> poly) || poly.empty()) {
        ROS_ERROR_STREAM("Not a valid off file (fill_hole): " << filename);
        return 0;
    }
    // Incrementally fill the holes
    unsigned int nb_holes = 0;

    BOOST_FOREACH(H h, halfedges(poly)) {
                    if (h->is_border()) {
                        std::vector<F> patch_facets;
                        std::vector<V> patch_vertices;

                        double perimeter = 0;

                        for (H hnext = h->next(); hnext !=
                                                  h; hnext = hnext->next()) //walk around halfedges of the hole to compute its perimeter
                        {
                            const Point3 &p = hnext->prev()->vertex()->point();
                            const Point3 &q = hnext->vertex()->point();
                            perimeter += CGAL::sqrt(CGAL::squared_distance(p, q));
                        }

                        const Point3 &p = h->prev()->vertex()->point();
                        const Point3 &q = h->vertex()->point();
                        perimeter += CGAL::sqrt(CGAL::squared_distance(p, q));

                        //std::cout<<"Hole perimeter: "<<perimeter << " threshold: "<<max_perimeter <<std::endl;

                        if (perimeter <= max_perimeter)

                            CGAL::Polygon_mesh_processing::triangulate_and_refine_hole(
                                    poly,
                                    h,
                                    std::back_inserter(patch_facets),
                                    std::back_inserter(patch_vertices));

                        ++nb_holes;
                    }
                }

    ROS_DEBUG_STREAM(nb_holes << " holes have been filled");

    std::ofstream out(mesh_path.c_str());
    out.precision(12);
    out << poly << std::endl;
    return 1;
}
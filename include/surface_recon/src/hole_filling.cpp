#include "hole_filling.hpp"

int fill_hole(std::string mesh_path, double max_perimeter)
{

 const char* filename = "temp2.off";
  std::ifstream input(filename);
  P poly;
  if ( !input || !(input >> poly) || poly.empty() ) {
    std::cerr << "Not a valid off file." << std::endl;
    return 0;
  }
  // Incrementally fill the holes
  unsigned int nb_holes = 0;

  
  BOOST_FOREACH(H h, halfedges(poly))
  {
    if(h->is_border())
    {
      std::vector<F>  patch_facets;
      std::vector<V> patch_vertices;

      double perimeter = 0;

    for (H hnext = h->next(); hnext != h; hnext = hnext->next()) //walk around halfedges of the hole to compute its perimeter
    {      
      const Point3& p = hnext->prev()->vertex()->point();
      const Point3& q = hnext->vertex()->point();
      perimeter+= CGAL::sqrt(CGAL::squared_distance(p, q));
    }

    const Point3& p = h->prev()->vertex()->point();
    const Point3& q = h->vertex()->point();
    perimeter+= CGAL::sqrt(CGAL::squared_distance(p, q));

    //std::cout<<"Hole perimeter: "<<perimeter << " threshold: "<<max_perimeter <<std::endl;

    if(perimeter <= max_perimeter)

        CGAL::Polygon_mesh_processing::triangulate_and_refine_hole(
                  poly,
                  h,
                  std::back_inserter(patch_facets),
                  std::back_inserter(patch_vertices) );

      ++nb_holes;
    }
  }


  std::cout << nb_holes << " holes have been filled" << std::endl;
  
  std::ofstream out(mesh_path.c_str());
  out.precision(12);
  out << poly << std::endl; 
  return 1;
}
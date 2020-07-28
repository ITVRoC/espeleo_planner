#include <iostream>
#include "recon_surface/surface_recon.hpp"
#include "recon_surface/pset.hpp"
#include "recon_surface/hole_filling.hpp"
#include "recon_surface/rm_artifacts.hpp"
#include "recon_surface/parser.hpp"
#include "recon_surface/texture.hpp"

//./PoissonRecon --in /media/sf_teste_SFM/cave_wnormals.ply --out /media/sf_teste_SFM/cave.screened.color.ply --depth 10 --color 16 --density

int main(int argc, char*argv[])
{  

 // const char* fname = (argc>1) ? argv[1] : "/home/guilherme/ITV_AR/data/s11d49_1cm.ply";
// /home/guilherme/ITV_AR/itv/point_cloud_vale/point_cloud/s11d49_1cm.txt

 map<string,string> args_map = parser(argc,argv).get_map();

 std::string version(std::to_string(CGAL_VERSION_NR));
 std::cout << "Using CGAL lib version " << version.substr(1,2) << "." << version.substr(3,2) <<std::endl; 


  std::vector<Point> points;
  std::vector<Vector> normals;
  std::vector<Color> colors; 
  std::list<PointVectorPair> estimated_pwn;
  std::vector<Point_3> orig_points;
  std::vector<unsigned int> indices;

  double hole_threshold = std::stod(args_map[string("--holemaxsize")]);
  double trim_threshold = std::stod(args_map[string("--trim")]);
  double sample_ratio = std::stod(args_map[string("--sample")]);
  double grid_multiplier = std::stod(args_map[string("--gridm")]);
  int texture_size = (int)std::stod(args_map[string("--texturesize")]);

  string input = (args_map[string("--ply")] != ".") ? args_map[string("--ply")] :  args_map[string("--csv")];
  string output = (args_map[string("--output")] == ".") ? input : args_map[string("--output")];

  FT average_spacing = 0.;

  Pset pointset(points,normals,colors,sample_ratio); // init my point cloud

  if(args_map[string("--ply")] != ".") // if its a .ply file
    pointset.read_ply(input);
  else
    pointset.read_csv(input);

  //pointset.translade_points_mean(); // translade points to centroid
  //pointset.smooth_pset(450);
  //pointset.correct_color(0.4); // normalize color illumination

  for(size_t i=0; i< points.size(); i++) //initialize structure for Kdtree
  {
    orig_points.push_back(Point_3(points[i][0],points[i][1],points[i][2]));
    indices.push_back(i);
  }


  // Initialize a spatial kd-tree for further search on original set of points
  Tree tree(
    boost::make_zip_iterator(boost::make_tuple( orig_points.begin(),indices.begin() )),
    boost::make_zip_iterator(boost::make_tuple( orig_points.end(),indices.end() ) )  
  );

  /////////////TESTING TEXTURE//////////////
  //texture t(tree,colors,output+"_textured.obj");
  //t.build_png();
  //t.save_obj();
  //return 0;
  //////////////////////////////////////////

  pointset.write_ply(output+"_full_pcd_rgb.ply"); //write full rgb .ply

  average_spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>(orig_points.begin(), orig_points.end(),6);

  double as_ratio = grid_multiplier*log(points.size()/10e4);//(points.size()/1200000.0)*3.0;

 // if(as_ratio > 6.0)
 //    as_ratio = 6.0;
  if(as_ratio < 0.6)
     as_ratio = 0.6;

  double cell_size = average_spacing*as_ratio; 



   std::cout <<"Estimated weight for grid size w.r.t. avg. point distance: "<<as_ratio<<std::endl;

   pointset.sample_points_cgal(cell_size,1); //0.02,1//sample points --

  //pointset.sample_points(0.25);

  std::cout<< "Sampled point set size: " << points.size()<<" | Normals: "<< normals.size()<<" | Original ply size: "<< colors.size() <<std::endl;
  std::cout<<"Success on sampling points."<<std::endl;
  std::cout<<"Estimating normals..."<<std::endl;



  //estimated_pwn = estimate_normals(points);
  if(normals.size() ==0)
    estimate_normals(points,estimated_pwn);//register_normals(points,estimate_normals(orig_points));
  else
    estimated_pwn = register_normals(points,grab_normals(orig_points,normals));

  printf("sizes: %ld, %ld\n", points.size(), estimated_pwn.size());

  //if(args_map[string("--csv")] != ".")
  cout<<"Writing point cloud .ply file... ";
    write_ply_wnormals(output+"_simplified_pcd_normals.ply", estimated_pwn,tree,colors);
  cout<<"Done."<<endl;
  //average_spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>(points.begin(), points.end(),5);

  trim_mesh(reconstruct_surface(estimated_pwn),tree, trim_threshold*(double)average_spacing);
  //parametrize_mesh("/media/sf_teste_SFM/reconstructed_and_trimmed.off","/media/sf_teste_SFM/reconstructed_and_trimmed_parametriz.eps");
  //pointset.write_ply(std::string("/home/guilherme/ITV_AR/data/cave_normals.ply"));
  std::cout << " Filling holes..." << std::endl;

  fill_hole("temp3.off",hole_threshold*(double)average_spacing);

  if(args_map[string("--output")] == ".")
  {
    texture t(tree,colors,output+"_textured.obj",texture_size);
    t.build_png();
    t.save_obj();
  }
  else
  {
    texture t(tree,colors,output+".obj",texture_size);
    t.build_png();
    t.save_obj();
  }

  //rm_artifacts_mesh(std::string("/media/sf_teste_SFM/reconstructed_trimmed_filled.off"),9);

  return EXIT_SUCCESS;

}

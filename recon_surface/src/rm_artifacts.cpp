#include "recon_surface/rm_artifacts.hpp"

bool check_pca_threshold(std::vector<Point_3> pts, float threshold, int max) {
    Eigen::MatrixXd points(max, 3);

    for (size_t i = 0; i < max; i++) {
        points(i, 0) = pts[i][0];
        points(i, 1) = pts[i][1];
        points(i, 2) = pts[i][2];
    }

    // Mean centering data.
    Eigen::VectorXd centroid = points.colwise().mean();

    points.rowwise() -= centroid.adjoint();

    Eigen::MatrixXd m = (points.adjoint() * points) / double(points.rows() - 1);

    //std::cout << "Here is the matrix m:" << std::endl << m << std::endl;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(m, Eigen::ComputeThinU | Eigen::ComputeThinV);
    /*
    std::cout << "Its singular values are:" << std::endl << svd.singularValues() << std::endl;
    std::cout << "Its left singular vectors are the columns of the thin U matrix:" << std::endl << svd.matrixU() << std::endl;
    std::cout << "Its right singular vectors are the columns of the thin V matrix:" << std::endl << svd.matrixV() << std::endl;
    Eigen::Vector3d rhs(1, 0, 0);
    std::cout << "Now consider this rhs vector:" << std::endl << rhs << std::endl;
    std::cout << "A least-squares solution of m*x = rhs is:" << std::endl << svd.solve(rhs) << std::endl;
    */

    if (svd.singularValues()[0] * threshold <= svd.singularValues()[2]) {
        //std::cout << "Its singular values are:" << std::endl << svd.singularValues() << std::endl;
        //getchar();
        return true;
    } else
        return false;
}

CPoint3 laplacian_smooth(std::vector<Point_3> pts) {
    Eigen::MatrixXd points(pts.size(), 3);
    for (size_t i = 0; i < pts.size(); i++) {
        points(i, 0) = pts[i][0];
        points(i, 1) = pts[i][1];
        points(i, 2) = pts[i][2];
    }
    Eigen::VectorXd centroid = points.colwise().mean();

    return CPoint3(centroid(0), centroid(1), centroid(2));
}

void rm_artifacts_mesh(std::string path, int k) {
    std::vector<Point_3> vertices;
    std::vector<unsigned int> indices;
    unsigned int pos = 0;
    unsigned int rmv_vtx = 0;

    std::ifstream in(path.c_str());

    Mesh m;
    in >> m;

    std::cout << "Removing artifacts from mesh" << std::endl;

    BOOST_FOREACH(vertex_descriptor vd, m.vertices()) {
                    vertices.push_back(Point_3(m.point(vd)[0], m.point(vd)[1], m.point(vd)[2]));
                    indices.push_back(pos++);
                }

    // Initialize a spatial kd-tree
    Tree tree(
            boost::make_zip_iterator(boost::make_tuple(vertices.begin(), indices.begin())),
            boost::make_zip_iterator(boost::make_tuple(vertices.end(), indices.end()))
    );

    BOOST_FOREACH(vertex_descriptor vd, m.vertices()) {
                    Point_3 p(m.point(vd)[0], m.point(vd)[1], m.point(vd)[2]);

                    K_neighbor_search search(tree, p, k * 3);

                    std::vector<Point_3> local_pts;

                    for (K_neighbor_search::iterator it = search.begin(); it != search.end(); it++) {
                        unsigned int idx = boost::get<1>(it->first);
                        local_pts.push_back(vertices[idx]);
                        //std::cout<< boost::get<0>(it->first)<< " " << boost::get<1>(it->first) << std::endl;
                    }

                    if (check_pca_threshold(local_pts, 0.10, k)) {
                        //m.remove_vertex(vd);
                        CPoint3 new_pt = laplacian_smooth(local_pts);
                        m.point(vd) = new_pt;
                        rmv_vtx++;

                    }


                }

/*
     BOOST_FOREACH(Mesh::Face_index face_index, m.faces())
    {
      Mesh::Halfedge_index he = m.halfedge(face_index);
      vertex_descriptor v0 = m.target(he);
      vertex_descriptor v1 = m.target(m.next(he));
      vertex_descriptor v2 = m.target(m.prev(he));

      if (m.is_removed(v0) || m.is_removed(v1) || m.is_removed(v2))
      {
        m.remove_face(face_index);
        rmv_vtx++;
      }
    }

   m.collect_garbage();
*/

    std::cout << "Smoothed " << rmv_vtx << " vertex considered artifacts." << std::endl;
    std::ofstream out("/media/sf_teste_SFM/reconstructed_trimmed_filled_cleaned.off");
    out << m;
}
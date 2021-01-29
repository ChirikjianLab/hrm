#include "include/MeshGenerator.h"

Mesh getMeshFromSQ(SuperQuadrics sq) {
    Eigen::Quaterniond quat;
    sq.setQuaternion(quat.setIdentity());
    sq.setPosition({0.0, 0.0, 0.0});

    Mesh M;
    ParametricPoints pts = getBoundary3D(sq);
    M = getMesh(pts);
    return M;
}

Mesh getMesh(const ParametricPoints& points) {
    Mesh res;

    std::list<Point> L;
    for (size_t i = 0; i < points.x.size(); i++) {
        L.push_front(Point(points.x[i], points.y[i], points.z[i]));
    }
    Triangulation T(L.begin(), L.end());
    assert(T.is_valid());

    // Get vertices
    std::map<Vertex_handle, int> index_of_vertex;

    Finite_vertices_iterator vit = T.finite_vertices_begin();
    Finite_vertices_iterator vdone = T.finite_vertices_end();

    if (vit == vdone) {
        std::cout << "no vertex" << std::endl;
    } else {
        int i = 0;
        while (vit != vdone) {
            Point auxp = vit->point();
            fcl::Vector3d auxv;
            auxv << auxp[0], auxp[1], auxp[2];
            res.vertices.push_back(auxv);
            index_of_vertex[vit.base()] = i;
            i++;
            ++vit;
        }
    }
    // Getting triangles
    Cell_handle c;
    Finite_cells_iterator cit = T.finite_cells_begin();
    Finite_cells_iterator cdone = T.finite_cells_end();

    if (cit == cdone) {
        std::cout << "no cells" << std::endl;
    } else {
        while (cit != cdone) {
            fcl::Triangle aux_tri0(index_of_vertex[cit->vertex(0)],
                                   index_of_vertex[cit->vertex(1)],
                                   index_of_vertex[cit->vertex(2)]);
            res.triangles.push_back(aux_tri0);
            fcl::Triangle aux_tri1(index_of_vertex[cit->vertex(0)],
                                   index_of_vertex[cit->vertex(2)],
                                   index_of_vertex[cit->vertex(3)]);
            res.triangles.push_back(aux_tri1);
            fcl::Triangle aux_tri2(index_of_vertex[cit->vertex(1)],
                                   index_of_vertex[cit->vertex(2)],
                                   index_of_vertex[cit->vertex(3)]);
            res.triangles.push_back(aux_tri2);
            fcl::Triangle aux_tri3(index_of_vertex[cit->vertex(0)],
                                   index_of_vertex[cit->vertex(1)],
                                   index_of_vertex[cit->vertex(3)]);
            res.triangles.push_back(aux_tri3);
            ++cit;
        }
    }

    return res;
}

ParametricPoints getBoundary3D(const SuperQuadrics& obj) {
    Eigen::MatrixXd sqMat = obj.getOriginShape();
    ParametricPoints X = getBoundaryFromMatrix(sqMat);
    return X;
}

// std::vector<SuperQuadrics> getSQFromCsv(const std::string& file_name,
//                                        const int num) {
//    // Read config file
//    std::vector<std::vector<double>> config = parse2DCsvFile(file_name);

//    // Generate SQ object
//    std::vector<SuperQuadrics> obj;
//    for (size_t j = 0; j < config.size(); j++) {
//        obj.emplace_back(
//            SuperQuadrics({config[j][0], config[j][1], config[j][2]},
//                          {config[j][3], config[j][4]},
//                          {config[j][5], config[j][6], config[j][7]},
//                          Eigen::Quaterniond(config[j][8], config[j][9],
//                                             config[j][10], config[j][11]),
//                          num));
//    }

//    return obj;
//}

ParametricPoints getBoundaryFromMatrix(const Eigen::MatrixXd& ptsMat) {
    ParametricPoints X;
    for (int i = 0; i < ptsMat.cols(); i++) {
        X.x.push_back(ptsMat(0, i));
        X.y.push_back(ptsMat(1, i));
        X.z.push_back(ptsMat(2, i));
    }

    return X;
}

MeshMatrix getMeshFromParamSurface(const Eigen::MatrixXd& surfBound,
                                   const int n) {
    auto Num = (n - 1) * (n - 1);
    Eigen::ArrayXd q((n - 1) * (n - 1));
    for (auto i = 0; i < n - 1; ++i) {
        q.segment(i * (n - 1), (n - 1)) =
            Eigen::ArrayXd::LinSpaced(n - 1, i * n, (i + 1) * n - 2);
    }

    MeshMatrix M;
    M.vertices = surfBound;

    M.faces = Eigen::MatrixXd::Zero(2 * Num, 3);
    M.faces.block(0, 0, Num, 1) = q;
    M.faces.block(0, 1, Num, 1) = q + n;
    M.faces.block(0, 2, Num, 1) = q + n + 1;
    M.faces.block(Num, 0, Num, 1) = q;
    M.faces.block(Num, 1, Num, 1) = q + 1;
    M.faces.block(Num, 2, Num, 1) = q + n + 1;

    return M;
}

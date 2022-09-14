#include "include/MeshGenerator.h"

hrm::Mesh hrm::getMeshFromSQ(const SuperQuadrics& sq) {
    const SuperQuadrics sqAux(sq.getSemiAxis(), sq.getEpsilon(),
                              {0.0, 0.0, 0.0}, Eigen::Quaterniond::Identity(),
                              sq.getNumParam());

    Mesh M;
    ParametricPoints pts = getBoundary3D(sqAux);
    M = getMesh(pts);
    return M;
}

hrm::Mesh hrm::getMesh(const ParametricPoints& points) {
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

hrm::ParametricPoints hrm::getBoundary3D(const SuperQuadrics& obj) {
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

hrm::ParametricPoints hrm::getBoundaryFromMatrix(const BoundaryPoints& ptsMat) {
    ParametricPoints X;
    for (int i = 0; i < ptsMat.cols(); i++) {
        X.x.push_back(ptsMat(0, i));
        X.y.push_back(ptsMat(1, i));
        X.z.push_back(ptsMat(2, i));
    }

    return X;
}

hrm::MeshMatrix hrm::getMeshFromParamSurface(const BoundaryPoints& surfBound,
                                             const Index n) {
    const auto numVtx = static_cast<Eigen::Index>(n);
    const auto numSurfVtx = (numVtx - 1) * (numVtx - 1);

    Eigen::ArrayXd q(numSurfVtx);
    for (auto i = 0; i < n - 1; ++i) {
        auto currIdx = static_cast<Eigen::Index>(i);

        q.segment(currIdx * (numVtx - 1), (numVtx - 1)) =
            Eigen::ArrayXd::LinSpaced(
                numVtx - 1, static_cast<double>(currIdx * numVtx),
                static_cast<double>((currIdx + 1) * numVtx - 2));
    }

    MeshMatrix M;
    M.vertices = surfBound;
    M.faces = Eigen::MatrixXd::Zero(2 * numSurfVtx, 3);
    M.faces.block(0, 0, numSurfVtx, 1) = q;
    M.faces.block(0, 1, numSurfVtx, 1) = q + numVtx;
    M.faces.block(0, 2, numSurfVtx, 1) = q + numVtx + 1;
    M.faces.block(numSurfVtx, 0, numSurfVtx, 1) = q;
    M.faces.block(numSurfVtx, 1, numSurfVtx, 1) = q + 1;
    M.faces.block(numSurfVtx, 2, numSurfVtx, 1) = q + numVtx + 1;

    return M;
}

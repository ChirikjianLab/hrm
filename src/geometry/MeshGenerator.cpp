#include "geometry/MeshGenerator.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_3.h>

#include <math.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using Triangulation = CGAL::Triangulation_3<K>;

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

    std::list<Triangulation::Point> L;
    for (size_t i = 0; i < points.x.size(); i++) {
        L.push_front(
            Triangulation::Point(points.x[i], points.y[i], points.z[i]));
    }
    Triangulation T(L.begin(), L.end());
    assert(T.is_valid());

    // Get vertices
    std::map<Triangulation::Vertex_handle, int> vertexIdx;

    Triangulation::Finite_vertices_iterator vit = T.finite_vertices_begin();
    Triangulation::Finite_vertices_iterator vdone = T.finite_vertices_end();

    if (vit == vdone) {
        std::cout << "no vertex" << std::endl;
    } else {
        int i = 0;
        while (vit != vdone) {
            Triangulation::Point auxp = vit->point();
            fcl::Vector3d auxv;
            auxv << auxp[0], auxp[1], auxp[2];
            res.vertices.push_back(auxv);
            vertexIdx[vit.base()] = i;
            i++;
            ++vit;
        }
    }

    // Get triangles
    Triangulation::Finite_cells_iterator cit = T.finite_cells_begin();
    Triangulation::Finite_cells_iterator cdone = T.finite_cells_end();

    if (cit == cdone) {
        std::cout << "no cells" << std::endl;
    } else {
        while (cit != cdone) {
            res.triangles.emplace_back(vertexIdx[cit->vertex(0)],
                                       vertexIdx[cit->vertex(1)],
                                       vertexIdx[cit->vertex(2)]);
            res.triangles.emplace_back(vertexIdx[cit->vertex(0)],
                                       vertexIdx[cit->vertex(2)],
                                       vertexIdx[cit->vertex(3)]);
            res.triangles.emplace_back(vertexIdx[cit->vertex(1)],
                                       vertexIdx[cit->vertex(2)],
                                       vertexIdx[cit->vertex(3)]);
            res.triangles.emplace_back(vertexIdx[cit->vertex(0)],
                                       vertexIdx[cit->vertex(1)],
                                       vertexIdx[cit->vertex(3)]);
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

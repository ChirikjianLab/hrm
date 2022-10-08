#pragma once

#include "SuperQuadrics.h"
#include "datastructure/DataType.h"
#include "util/Parse2dCsvFile.h"

#include <fcl/fcl.h>

namespace hrm {

/** \brief Mesh as a structure of vertices and triangles */
struct Mesh {
    /** \brief List of fcl::Vector3d object for vertices */
    std::vector<fcl::Vector3d> vertices;

    /** \brief List of fcl::Triangle object for faces */
    std::vector<fcl::Triangle> triangles;
};

/** \brief Mesh with vertices and faces stored in MatrixXd format */
struct MeshMatrix {
    /** \brief List of vertices */
    BoundaryPoints vertices;

    /** \brief Faces in the matrix format */
    Eigen::MatrixXd faces;
};

/** \brief Vectors of point coordinates */
struct ParametricPoints {
    /** \brief x-coordinate of the surface points */
    std::vector<Coordinate> x;

    /** \brief y-coordinate of the surface points */
    std::vector<Coordinate> y;

    /** \brief z-coordinate of the surface points */
    std::vector<Coordinate> z;
};

/** \brief Get mesh info from SuperQuadrics class */
Mesh getMeshFromSQ(const SuperQuadrics& sq);

/** \brief Get mesh info from 3D point cloud */
Mesh getMesh(const ParametricPoints& points);

/** \brief Get 3D point cloud from SuperQuadric class */
ParametricPoints getBoundary3D(const SuperQuadrics& obj);

ParametricPoints getBoundaryFromMatrix(const BoundaryPoints& ptsMat);

MeshMatrix getMeshFromParamSurface(const BoundaryPoints& surfBound,
                                   const Index n);

}  // namespace hrm

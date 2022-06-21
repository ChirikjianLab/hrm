#pragma once

#include "SuperQuadrics.h"
#include "datastructure/include/DataType.h"
#include "util/include/Parse2dCsvFile.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_3.h>
#include <fcl/fcl.h>

#include <math.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using Triangulation = CGAL::Triangulation_3<K>;

using Finite_vertices_iterator = Triangulation::Finite_vertices_iterator;
using Cell_handle = Triangulation::Cell_handle;
using Vertex_handle = Triangulation::Vertex_handle;
using Locate_type = Triangulation::Locate_type;
using Point = Triangulation::Point;

using Finite_cells_iterator = Triangulation::Finite_cells_iterator;
using Point_3 = K::Point_3;

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
Mesh getMeshFromSQ(SuperQuadrics sq);

/** \brief Get mesh info from 3D point cloud */
Mesh getMesh(const ParametricPoints& points);

/** \brief Get 3D point cloud from SuperQuadric class */
ParametricPoints getBoundary3D(const SuperQuadrics& obj);

/** \brief Generate SuperQuadrics class from configuration .csv file */
std::vector<SuperQuadrics> getSQFromCsv(const std::string& file_name,
                                        const Index num);

ParametricPoints getBoundaryFromMatrix(const BoundaryPoints& ptsMat);

MeshMatrix getMeshFromParamSurface(const BoundaryPoints& surfBound,
                                   const Index n);

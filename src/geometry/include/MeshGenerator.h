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
    std::vector<fcl::Vector3d> vertices;
    std::vector<fcl::Triangle> triangles;
};

/** \brief Mesh with vertices and faces stored in MatrixXd format */
struct MeshMatrix {
    BoundaryPoints vertices;
    Eigen::MatrixXd faces;
};

/** \brief ParametricPoints vectors of point coordinates */
struct ParametricPoints {
    std::vector<Coordinate> x;
    std::vector<Coordinate> y;
    std::vector<Coordinate> z;
};

/** \brief get mesh info from SuperQuadrics class */
Mesh getMeshFromSQ(SuperQuadrics sq);

/** \brief get mesh info from 3D point cloud */
Mesh getMesh(const ParametricPoints& points);

/** \brief get 3D point cloud from SuperQuadric class */
ParametricPoints getBoundary3D(const SuperQuadrics& obj);

/** \brief Generate SuperQuadrics class from configuration .csv file */
std::vector<SuperQuadrics> getSQFromCsv(const std::string& file_name,
                                        const Index num);

ParametricPoints getBoundaryFromMatrix(const BoundaryPoints& ptsMat);

MeshMatrix getMeshFromParamSurface(const BoundaryPoints& surfBound,
                                   const Index n);

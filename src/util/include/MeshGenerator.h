#ifndef MESH_GENERATOR_H
#define MESH_GENENATOR_H

#include "Parse2dCsvFile.h"
#include "src/geometry/include/SuperQuadrics.h"

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

struct EMesh {
    std::vector<fcl::Vector3d> vertices;
    std::vector<fcl::Triangle> triangles;
};

struct ParametricPoints {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
};

/*
 * \brief get mesh info from SuperQuadrics class
 */
EMesh getMeshFromSQ(SuperQuadrics sq);

/*
 * \brief get mesh info from 3D point cloud
 */
EMesh getMesh(const ParametricPoints& points);

/*
 * \brief get 3D point cloud from SuperQuadric class
 */
ParametricPoints getBoundary3D(const SuperQuadrics& obj);

/*
 * \brief Generate SuperQuadrics class from configuration .csv file
 */
std::vector<SuperQuadrics> getSQFromCsv(const std::string& file_name,
                                        const int num);

#endif  // MESH_GENERATOR_H

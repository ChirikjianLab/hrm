#ifndef MESH_GENERATOR_H
#define MESH_GENENATOR_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_3.h>
#include <fcl/fcl.h>

#include <vector>
#include <iostream>
#include <fstream>
#include <math.h>
#include <sstream>
#include <string>

#include "src/geometry/superquadrics.h"

using namespace std;

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_3<K>      Triangulation;

typedef Triangulation::Finite_vertices_iterator Finite_vertices_iterator;
typedef Triangulation::Cell_handle    Cell_handle;
typedef Triangulation::Vertex_handle  Vertex_handle;
typedef Triangulation::Locate_type    Locate_type;
typedef Triangulation::Point          Point;

typedef Triangulation::Finite_cells_iterator Finite_cells_iterator;
typedef K::Point_3                                           Point_3;

struct EMesh
{
    std::vector<fcl::Vector3d> vertices;
    std::vector<fcl::Triangle> triangles;
};

struct ParametricPoints
{
    std::vector<double>x;
    std::vector<double>y;
    std::vector<double>z;
};

class MeshGenerator{
    public:
        MeshGenerator();
        EMesh getMesh(ParametricPoints points_);
        ParametricPoints getBoundary3D(SuperQuadrics obj);
};

#endif // MESH_GENERATOR_H

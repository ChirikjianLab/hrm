#include "include/MeshGenerator.h"

MeshGenerator::MeshGenerator() {}

EMesh MeshGenerator::getMesh(ParametricPoints points_) {
  EMesh res;

  std::list<Point> L;
  for (size_t i = 0; i < points_.x.size(); i++) {
    L.push_front(Point(points_.x[i], points_.y[i], points_.z[i]));
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

ParametricPoints MeshGenerator::getBoundary3D(SuperQuadrics obj) {
  ParametricPoints X;
  for (int i = 0; i < obj.getNum(); i++) {
    X.x.push_back(obj.getOriginShape()(0, i));
    X.y.push_back(obj.getOriginShape()(1, i));
    X.z.push_back(obj.getOriginShape()(2, i));
  }
  return X;
}

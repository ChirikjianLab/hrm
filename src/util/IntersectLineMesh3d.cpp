#include "include/IntersectLineMesh3d.h"
#include <iostream>

std::vector<Eigen::Vector3d> IntersectLineMesh3d::intersect(
    const Eigen::VectorXd& line, const MeshMatrix& shape) {
    std::vector<Eigen::Vector3d> points;

    if (line(0) > shape.vertices.row(0).maxCoeff() ||
        line(0) < shape.vertices.row(0).minCoeff()) {
        return points;
    }
    if (line(1) > shape.vertices.row(1).maxCoeff() ||
        line(1) < shape.vertices.row(1).minCoeff()) {
        return points;
    }

    double tol = 1e-12;
    Eigen::Vector3d t0, u, v, n, pt;
    double a, b, uu, uv, vv, wu, wv, D, s, t;

    /*
     * \brief Specifially for vertical sweep lines, first do a quick check
     * according to x and y coord: If the x or y coordinates of the vertical
     * sweep line is out of range of the triangle, directly ignore
     */
    for (auto i = 0; i < shape.faces.rows(); ++i) {
        // ignore the face that is out of range
        if (line(0) <
                std::fmin(
                    shape.vertices(0, int(shape.faces(i, 0))),
                    std::fmin(shape.vertices(0, int(shape.faces(i, 1))),
                              shape.vertices(0, int(shape.faces(i, 2))))) ||
            line(0) >
                std::fmax(
                    shape.vertices(0, int(shape.faces(i, 0))),
                    std::fmax(shape.vertices(0, int(shape.faces(i, 1))),
                              shape.vertices(0, int(shape.faces(i, 2)))))) {
            continue;
        }
        if (line(1) <
                std::fmin(
                    shape.vertices(1, int(shape.faces(i, 0))),
                    std::fmin(shape.vertices(1, int(shape.faces(i, 1))),
                              shape.vertices(1, int(shape.faces(i, 2))))) ||
            line(1) >
                std::fmax(
                    shape.vertices(1, int(shape.faces(i, 0))),
                    std::fmax(shape.vertices(1, int(shape.faces(i, 1))),
                              shape.vertices(1, int(shape.faces(i, 2)))))) {
            continue;
        }

        // find triangle edge vectors
        t0 = shape.vertices.col(int(shape.faces(i, 0)));
        u = shape.vertices.col(int(shape.faces(i, 1))) - t0;
        v = shape.vertices.col(int(shape.faces(i, 2))) - t0;

        // triangle normal
        n = u.cross(v);
        n.normalize();

        // vector between triangle origin and line origin
        a = -n.dot(line.head(3) - t0);
        b = n.dot(line.tail(3));
        if (!((std::fabs(b) > tol) && (n.norm() > tol))) {
            continue;
        }

        /* \brief Compute intersection point of line with supporting plane:
                  If pos = a/b < 0: point before ray
                  IF pos = a/b > |dir|: point after edge*/
        // coordinates of intersection point
        pt = line.head(3) + a / b * line.tail(3);

        // Test if intersection point is inside triangle
        // normalize direction vectors of triangle edges
        uu = u.dot(u);
        uv = u.dot(v);
        vv = v.dot(v);

        // coordinates of vector v in triangle basis
        wu = u.dot(pt - t0);
        wv = v.dot(pt - t0);

        // normalization constant
        D = pow(uv, 2) - uu * vv;

        // test first coordinate
        s = (uv * wv - vv * wu) / D;
        if ((s < -tol) || (s > 1.0 + tol)) {
            continue;
        }

        // test second coordinate and third triangle edge
        t = (uv * wu - uu * wv) / D;
        if ((t < -tol) || (s + t > 1.0 + tol)) {
            continue;
        }

        // keep only interesting points
        points.push_back(pt);
        if (points.size() == 2) {
            break;
        }
    }

    return points;
}

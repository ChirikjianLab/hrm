#include "intersectlinemesh3d.h"

vector<Vector3d> intersectLineMesh3d::intersect(VectorXd line, MatrixXd vertices, MatrixXd faces){
    vector<Vector3d> points;
    double tol = 1e-12;

    for (int i=0; i<faces.rows(); i++) {
        // find triangle edge vectors
        Vector3d t0 = vertices.col(int(faces(i,0))),
                 u = vertices.col(int(faces(i,1))),
                 v = vertices.col(int(faces(i,2)));
        u -= t0; v -= t0;

        // triangle normal
        Vector3d n = u.cross(v);
        n.normalize();

        // direction vector of line
        Vector3d dir = line.tail(3);

        // vector between triangle origin and line origin
        Vector3d w0 = line.head(3) - t0;
        double a = -n.dot(w0), b = n.dot(dir);
        if(!((fabs(b) > tol) && (n.norm() > tol))) continue;

        /* compute intersection point of line with supporting plane
           If pos < 0: point before ray
           IF pos > |dir|: point after edge*/
        double pos = a / b;

        // coordinates of intersection point
        Vector3d pt = line.head(3) + pos*dir;

        // Test if intersection point is inside triangle
        // normalize direction vectors of triangle edges
        double uu = u.dot(u), uv = u.dot(v), vv = v.dot(v);

        // coordinates of vector v in triangle basis
        Vector3d w = pt - t0;
        double wu = w.dot(u), wv = w.dot(v);

        // normalization constant
        double D = pow(uv,2) - uu * vv;

        // test first coordinate
        double s = (uv*wv - vv*wu) / D;
        if((s < -tol) || (s > 1.0+tol)) continue;

        // test second coordinate and third triangle edge
        double t = (uv*wu - uu*wv) / D;
        if((t < -tol) || (s+t > 1.0+tol)) continue;

        // keep only interesting points
        points.push_back(pt);
    }

    return points;
}

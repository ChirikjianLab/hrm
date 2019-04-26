#include "intersectlinemesh3d.h"

vector<Vector3d> intersectLineMesh3d::intersect(VectorXd line, Matrix3Xd vertices, MatrixX3d faces){
    vector<Vector3d> points;

    if( line(0) > vertices.row(0).maxCoeff() || line(0) < vertices.row(0).minCoeff() ) return points;
    if( line(1) > vertices.row(1).maxCoeff() || line(1) < vertices.row(1).minCoeff() ) return points;

    double tol = 1e-12;

    Vector3d t0,u,v,n,
             pt;
    double a,b,uu,uv,vv,wu,wv,D,s,t;

    for (int i=0; i<faces.rows(); i++) {
        // find triangle edge vectors
        t0 = vertices.col(int(faces(i,0)));
        u = vertices.col(int(faces(i,1))) - t0;
        v = vertices.col(int(faces(i,2))) - t0;

        // triangle normal
        n = u.cross(v); n.normalize();

        // vector between triangle origin and line origin
        a = -n.dot(line.head(3) - t0);
        b = n.dot(line.tail(3));
        if(!((fabs(b) > tol) && (n.norm() > tol))) continue;

        /* compute intersection point of line with supporting plane
           If pos = a/b < 0: point before ray
           IF pos = a/b > |dir|: point after edge*/
        // coordinates of intersection point
        pt = line.head(3) + a/b*line.tail(3);

        // Test if intersection point is inside triangle
        // normalize direction vectors of triangle edges
        uu = u.dot(u);
        uv = u.dot(v);
        vv = v.dot(v);

        // coordinates of vector v in triangle basis
        wu = u.dot(pt - t0);
        wv = v.dot(pt - t0);

        // normalization constant
        D = pow(uv,2) - uu * vv;

        // test first coordinate
        s = (uv*wv - vv*wu) / D;
        if((s < -tol) || (s > 1.0+tol)) continue;

        // test second coordinate and third triangle edge
        t = (uv*wu - uu*wv) / D;
        if((t < -tol) || (s+t > 1.0+tol)) continue;

        // keep only interesting points
        points.push_back(pt);
        if(points.size() == 2) break;
    }

    return points;
}

vector<Vector3d> intersectLineMesh3d::intersect_mat(VectorXd line, Matrix3Xd vertices, MatrixX3d faces){
    vector<Vector3d> points;

    if( line(0) > vertices.row(0).maxCoeff() || line(0) < vertices.row(0).minCoeff() ) return points;
    if( line(1) > vertices.row(1).maxCoeff() || line(1) < vertices.row(1).minCoeff() ) return points;

    double tol = 1e-12;

    Matrix3Xd t0=MatrixXd::Constant(3,faces.rows(),1),u=t0,v=t0,n=t0,pt=t0;
    RowVectorXd a,b,uu,uv,vv,wu,wv,D,s,t;

    RowVectorXd Valid, ind1, ind2;

    for (int i=0; i<faces.rows(); i++) {
        // find triangle edge vectors
        t0.col(i) = vertices.col(int(faces(i,0)));
        u.col(i) = vertices.col(int(faces(i,1))) - t0.col(i);
        v.col(i) = vertices.col(int(faces(i,2))) - t0.col(i);

        // triangle normal
        n.col(i) = u.col(i).cross(v.col(i));
        n.col(i).normalize();
    }

//    // vector between triangle origin and line origin
//    a = -(u.transpose() * (line.head(3) - t0)).diagonal();
//    b = (u.transpose() * line.tail(3)).diagonal();

//    Valid = (b.array() > tol).select(1,0);
//    Valid = (n.array().square().rowwise().sum() > tol).select();

//    /* compute intersection point of line with supporting plane
//           If pos = a/b < 0: point before ray
//           IF pos = a/b > |dir|: point after edge*/
//    // coordinates of intersection point
//    pt = line.head(3).array() + a.array() / b.array() * line.tail(3).array();

//    // Test if intersection point is inside triangle
//    // normalize direction vectors of triangle edges
//    uu = (u.transpose() * u).diagonal();
//    uv = (u.transpose() * v).diagonal();
//    vv = (v.transpose() * v).diagonal();

//    // coordinates of vector v in triangle basis
//    wu = (u.transpose() * (pt-t0)).diagonal();
//    wv = (v.transpose() * (pt-t0)).diagonal();

//    // normalization constant
//    D = uv.array().square() - uu.array() * vv.array();

//    // test first coordinate
//    s = (uv.array() * wv.array() - vv.array() * wu.array()).array() / D.array();
//    ind1 = (!(s.array() < -tol.array()) || (s.array() > 1.0+tol.array()));

//    // test second coordinate and third triangle edge
//    t = (uv.array() * wu.array() - uu.array() * wv.array()).array() / D.array();
//    ind2 = (!(t.array() < -tol.array()) || ((s+t).array() > 1.0+tol.array()));

//    // keep only interesting points
//    points.push_back(pt);

    return points;
}

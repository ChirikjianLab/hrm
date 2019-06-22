#include "multibodytree3d.h"

void multibodytree3D::addBase(SuperQuadrics base){
    Base = base;
}

void multibodytree3D::addBody(SuperQuadrics link){
    // Add link
    Link.push_back(link);
    numLinks ++;

    // Add tranformation related to Base
    Matrix4d g; g.setIdentity();
    g.block<3,3>(0,0) = link.Shape.q.toRotationMatrix();
    g.block<3,1>(0,3) = Vector3d(link.Shape.pos);
    tf.push_back(g);
}

void multibodytree3D::robotTF(Matrix4d g){
    for(int i=0; i<3; i++) Base.Shape.pos[i] = g.row(3)[i];
    Matrix3d mat = g.block<3,3>(0,0); Quaterniond quat(mat);
    Base.Shape.q = quat;

    Matrix4d g_Link;
    for (size_t i=0; i<numLinks; i++) {
        g_Link = g * tf[i];
        for(int j=0; j<3; j++) Link[i].Shape.pos[j] = g.row(3)[j];
        mat = g_Link.block<3,3>(0,0); Quaterniond quat(mat);
        Link[i].Shape.q = quat;
    }
}

vector<MatrixXd> multibodytree3D::minkSumSQ(SuperQuadrics S1, int k){
    vector<MatrixXd> Mink;

    // Minkowski sums for Base
    Mink.push_back(S1.minkSum3D(Base.Shape, k));
    Matrix3d R_base = Base.Shape.q.toRotationMatrix(), R_Link;

    // Minkowski sums for Links
    Vector3d link_tc;
    for (size_t i=0; i<numLinks; i++) {
        R_Link = R_base * tf[i].block<3,3>(0,0); Quaterniond quat(R_Link);
        Link[i].Shape.q = quat;
        link_tc = R_base * tf[i].block<3,1>(0,3);

        Mink.push_back(S1.minkSum3D(Link[i].Shape,k).colwise() - link_tc);
    }

    return Mink;
}

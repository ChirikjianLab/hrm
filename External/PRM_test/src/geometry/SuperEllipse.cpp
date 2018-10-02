#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <fcl/fcl.h>
#include "fcl/geometry/shape/ellipsoid.h"
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/collision.h>

#include <unsupported/Eigen/Polynomials>

#include "geometry/SuperEllipse.h"

using namespace Eigen;
using namespace std;

MatrixXd SuperEllipse::originShape(double a[6], int num){
    double th;
    Vector2d x;
    MatrixXd trans(2,num), X(2,num);

    for(int i=0; i<num; i++){   
        th = 2*i*pi_val/(num-1);
        x(0,0) = a[0] * expFun(th,a[3],0);
        x(1,0) = a[1] * expFun(th,a[3],1);
        trans(0,i) = a[4]; trans(1,i) = a[5];
        X(0,i) = x(0,0); X(1,i) = x(1,0);
    }
    X = Rotation2Dd(a[2]).matrix() * X + trans;
    return X;
}
    
double SuperEllipse::expFun(double th, double p, bool func){
    return (func == 0) ? sgn(cos(th)) * pow(abs(cos(th)), p) : sgn(sin(th)) * pow(abs(sin(th)), p) ;
}

Matrix2d SuperEllipse::rotation_angle_axis(double theta){
    Matrix2d S;
    S << 0, -1, 1,0;
    Matrix2d R;
    MatrixXd id = MatrixXd::Identity(2,2);
    R = id + ((sin(theta)) * S) + ((1 - cos(theta))*(S * S));
    return R;
}

void SuperEllipse::printTrianglesVertices(){
  cout << "Vertices"<<endl;
  for(int i=0; i<vertices.size();i++){
    cout << vertices[i](0,0) << " " << vertices[i](1,0) << " " << vertices[i](2,0) << endl; 
  }
}

bool SuperEllipse::readingVerticesNTriangles(std::string file_vertices, std::string file_triangles){        
    //Getting the vertices and triangles from files
    std::fstream inputvertices(file_vertices,std::ios_base::in);
    if(!inputvertices){
        std::cout << "Error opening vertices file" << std::endl;
        return(1);
    }    
    double v1, v2;
    while(inputvertices >> v1 >> v2){
        fcl::Vector3<double> aux_vec;
        aux_vec << v1,v2,0.0;
        vertices.push_back(aux_vec);
    }
    inputvertices.close();

    std::fstream inputtriangles(file_triangles,std::ios_base::in);
    if(!inputtriangles){
        std::cout << "Error opening triangle file" << std::endl;
        return(1);
    }    
    int t1, t2, t3;
    while(inputtriangles >> t1 >> t2 >> t3){
        fcl::Triangle aux_tri(t1,t2,t3);
        triangles.push_back(aux_tri);
    }
    inputtriangles.close();
    return true;
}



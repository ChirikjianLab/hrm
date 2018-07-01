#include "ptinpoly.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

bool ptInPoly::isPtInPoly(polyCSpace polyVtx, vector<double> pt){
    bool flag = 0;
    Matrix4d invMat;
    Vector4d p, alpha;

    // convert data structure and compute \alpha for each simplex
    p << pt[0], pt[1], pt[2], 1;

    int n = polyVtx.invMat.size();
    for(int i=0; i<n; i++){
        invMat = Map<Matrix4d, Unaligned>(polyVtx.invMat[i].data(), 4, 4);
        alpha = invMat * p;
        if(alpha.minCoeff()>=0 && alpha.maxCoeff()<=1){
            flag = 1;
            return flag;
        }
    }

    return flag;
}

#include <iostream>
#include <list>
#include <src/planners/highwayroadmap.h>

#include <fstream>

#define pi 3.1415926

highwayRoadmap::highwayRoadmap(SuperEllipse robot, double endpt[2][2], SuperEllipse* arena, SuperEllipse* obs, option opt){
    this->Robot = robot;
    this->Endpt = *endpt;
    this->Arena = arena;
    this->Obs = obs;
    this->N_o = opt.N_o;
    this->N_s = opt.N_s;

    this->infla = opt.infla;
    this->N_layers = opt.N_layers;
    this->N_dy = opt.N_dy;
    this->N_KCsample = opt.sampleNum;

    this->Cost = 0;
}

void highwayRoadmap::multiLayers(){
    // angle steps
    double dr = pi/(this->N_layers-1);
    graph multiGraph;

    for(int i=0; i<this->N_layers; i++){
        this->Robot.a[2] = dr*i;
        // boundary for obstacles and arenas
        boundary bd = this->boundaryGen();

        // collision-free cells, stored by ty, xL, xU, xM
        cf_cell CFcell = this->rasterScan(bd.bd_s, bd.bd_o);

        // construct adjacency matrix for one layer
        this->oneLayer(CFcell);
    }
}

boundary highwayRoadmap::boundaryGen(){
    SuperEllipse robot_infla = this->Robot;
    boundary bd;

    // Enlarge the robot
    robot_infla.a[0] *= 1+this->infla;
    robot_infla.a[1] *= 1+this->infla;

    // calculate Minkowski boundary points
    for(int i=0; i<this->N_s; i++){
        bd.bd_s.push_back( this->Arena[i].minkSum2D(this->Arena[i].a, robot_infla.a, this->Arena[i].num, -1) );
    }
    for(int i=0; i<this->N_o; i++){
        bd.bd_o.push_back( this->Obs[i].minkSum2D(this->Obs[i].a, robot_infla.a, this->Obs[i].num, +1) );
    }

    return bd;
}

cf_cell highwayRoadmap::rasterScan(vector<MatrixXd> bd_s, vector<MatrixXd> bd_o){
    boundary::sepBd P_bd_s[this->N_s], P_bd_o[this->N_o];
    boundary::sepBd x_bd_s[this->N_s], x_bd_o[this->N_o];

    // Separate boundaries of Arenas and Obstacles into two parts
    for(int i=0; i< this->N_s; i++){
        P_bd_s[i] = this->separateBoundary(bd_s[i]);
    }
    for(int i=0; i< this->N_o; i++){
        P_bd_o[i] = this->separateBoundary(bd_o[i]);
    }

    // Find closest points for each raster scan line
    double ty, dy = (P_bd_s[0].max_y-P_bd_s[0].min_y)/this->N_dy;
    for(int i=0; i< this->N_dy; i++){
        // y-coordinate of each sweep line
        ty = P_bd_s[0].min_y + (i-1) * dy;
        cout << ty << endl;

        for(int j=0; j< this->N_s; j++){
            x_bd_s[j] = this->closestPt(P_bd_s[j], ty);
            cout << x_bd_s[j].x_L << ' ' << x_bd_s[j].x_R << endl;
        }
        for(int j=0; j< this->N_o; j++){
            x_bd_o[j] = this->closestPt(P_bd_o[j], ty);
            cout << x_bd_o[j].x_L << ' ' << x_bd_o[j].x_R << endl;
        }
    }

}

void highwayRoadmap::oneLayer(cf_cell CFcell){

    /*

this->vtxEdge.vertex.push_back({});
this->vtxEdge.edge.push_back(make_pair());

this->N_v_layers.pushback(sizeof(this->vtxEdge.vertex)/sizeof(this->vtxEdge.vertex[0]));*/
}

// For a given curve, separate its boundary into two parts
boundary::sepBd highwayRoadmap::separateBoundary(MatrixXd bd){
    const int half_num = this->Arena[0].num/2;

    MatrixXd::Index I_max_y, I_min_y;
    int I_start_y;
    MatrixXd P_bd_L, P_bd_R;
    double max_y, min_y;

    boundary::sepBd P_bd;

    max_y = bd.row(1).maxCoeff(&I_max_y);
    min_y = bd.row(1).minCoeff(&I_min_y);
    I_start_y = min(I_max_y, I_min_y);
    I_start_y = min(I_start_y, half_num);

    P_bd_L.setZero(2,half_num);
    P_bd_L = bd.block(0, I_start_y, 2, half_num);

    P_bd_R.setZero(2,half_num);
    P_bd_R.topLeftCorner(2, I_start_y) = bd.topLeftCorner(2, I_start_y);
    P_bd_R.bottomRightCorner(2, half_num-I_start_y) = bd.bottomRightCorner(2, half_num-I_start_y);

    P_bd.P_bd_L = P_bd_L;
    P_bd.P_bd_R = P_bd_R;
    P_bd.max_y = max_y;
    P_bd.min_y = min_y;

    return P_bd;
}

boundary::sepBd highwayRoadmap::closestPt(boundary::sepBd P_bd, double ty){
    boundary::sepBd x_bd;
    MatrixXd::Index I_L, I_R;
    VectorXd y(1);

    if( (ty > P_bd.max_y) || (ty < P_bd.min_y) ){
        x_bd.x_L = numeric_limits<double>::quiet_NaN();
        x_bd.x_R = numeric_limits<double>::quiet_NaN();
        return x_bd;
    }

    y << ty;
    (P_bd.P_bd_L.row(1).colwise() - y).colwise().squaredNorm().minCoeff(&I_L);
    (P_bd.P_bd_R.row(1).colwise() - y).colwise().squaredNorm().minCoeff(&I_R);

    x_bd.x_L = P_bd.P_bd_L(0,I_L);
    x_bd.x_R = P_bd.P_bd_R(0,I_R);
    return x_bd;
}

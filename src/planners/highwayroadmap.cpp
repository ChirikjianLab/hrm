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
    boundary::sepBd x_bd_s[this->N_dy][this->N_s], x_bd_o[this->N_dy][this->N_o];

    MatrixXd bd_s_L[this->N_s], bd_s_R[this->N_s], bd_o_L[this->N_o], bd_o_R[this->N_o];
    MatrixXd x_s_L(this->N_dy, this->N_s), x_s_R(this->N_dy, this->N_s);
    MatrixXd x_o_L(this->N_dy, this->N_o), x_o_R(this->N_dy, this->N_o);

    // Separate boundaries of Arenas and Obstacles into two parts
    for(int i=0; i< this->N_s; i++) {
        P_bd_s[i] = this->separateBoundary(bd_s[i]);
        bd_s_L[i] = P_bd_s[i].P_bd_L;
        bd_s_R[i] = P_bd_s[i].P_bd_R;
    }
    for(int i=0; i< this->N_o; i++) {
        P_bd_o[i] = this->separateBoundary(bd_o[i]);
        bd_o_L[i] = P_bd_o[i].P_bd_L;
        bd_o_R[i] = P_bd_o[i].P_bd_R;
    }

    // Find closest points for each raster scan line
    double ty[this->N_dy], dy = (P_bd_s[0].max_y-P_bd_s[0].min_y)/this->N_dy;
    for(int i=0; i< this->N_dy; i++){
        // y-coordinate of each sweep line
        ty[i] = P_bd_s[0].min_y + (i-1) * dy;
        // x-coordinate of the intersection btw sweep line and arenas
        for(int j=0; j< this->N_s; j++) {
            x_bd_s[i][j] = this->closestPt(P_bd_s[j], ty[i]);
            x_s_L(i,j) = x_bd_s[i][j].x_L;
            x_s_R(i,j) = x_bd_s[i][j].x_R;
        }
        // x-coordinate of the intersection btw sweep line and obstacles
        for(int j=0; j< this->N_o; j++) {
            x_bd_o[i][j] = this->closestPt(P_bd_o[j], ty[i]);
            x_o_L(i,j) = x_bd_o[i][j].x_L;
            x_o_R(i,j) = x_bd_o[i][j].x_R;
        }
    }

    cout << x_o_L << ' xxx ' << x_o_R << endl;

    // Enlarge the obstacle to form convex CF cells
    x_o_L = this->boundaryEnlarge(bd_o_L, x_o_L, ty, -1);
    x_o_R = this->boundaryEnlarge(bd_o_R, x_o_R, ty, +1);
    cout << x_o_L << ' xxx ' << x_o_R << endl;
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

    // Find separating point
    max_y = bd.row(1).maxCoeff(&I_max_y);
    min_y = bd.row(1).minCoeff(&I_min_y);
    I_start_y = min(I_max_y, I_min_y);
    I_start_y = min(I_start_y, half_num);

    // Left part
    P_bd_L.setZero(2,half_num);
    P_bd_L = bd.block(0, I_start_y, 2, half_num);
    // Right part
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

    // check if ty in the range of each arena/obstacle
    if( (ty > P_bd.max_y) || (ty < P_bd.min_y) ){
        x_bd.x_L = numeric_limits<double>::quiet_NaN();
        x_bd.x_R = numeric_limits<double>::quiet_NaN();
        return x_bd;
    }
    // For each ty, find closes point, ie the intersection btw sweep line and arena/obstacle
    y << ty;
    (P_bd.P_bd_L.row(1).colwise() - y).colwise().squaredNorm().minCoeff(&I_L);
    (P_bd.P_bd_R.row(1).colwise() - y).colwise().squaredNorm().minCoeff(&I_R);

    x_bd.x_L = P_bd.P_bd_L(0,I_L);
    x_bd.x_R = P_bd.P_bd_R(0,I_R);
    return x_bd;
}

MatrixXd highwayRoadmap::boundaryEnlarge(MatrixXd bd_o[], MatrixXd x_o, double ty[], int K){
    MatrixXd x_o_Ex(this->N_dy, this->N_o);
    double x_Ex;
    double d;

    for(int j=0; j< this->N_o; j++){
        int count = 0;

        for(int i=0; i< this->N_dy-1; i++){
            double dist = 0, phi;
            x_o_Ex(i,j) = numeric_limits<double>::quiet_NaN();

            if( isnan(x_o(i,j)) || isnan(x_o(i+1,j)) ) continue;
            count += 1;
            double p1[2] = {x_o(i,j), ty[i]};
            double p2[2] = {x_o(i+1,j), ty[i+1]};

            // Search for farthest point to the line segment def by two intersecting points
            for(int k=0; k< sizeof(bd_o[j].rows()); k++){
                double p[2] = {bd_o[j](0,k), bd_o[j](1,k)};

                if( (p[1] > ty[i]) && (p[1] < ty[i+1]) ){
                    d = abs( (p2[1]-p1[1])*p[0] - (p2[0]-p1[0])*p[1] + p2[0]*p1[1] - p2[1]*p1[0] )
                            / sqrt( pow((p2[1]-p2[1]),2) + pow((p2[0]-p1[0]),2) );
                    if(d > dist) dist = d;
                }
            }
            phi = atan2( p2[1]-p1[2], p2[0]-p1[0] );
            x_Ex = x_o(i,j) + K * dist/sin(phi);

            // Update the boundary point to be farthest to the previous one
            if(K == -1) {if(x_Ex <= x_o_Ex(i,j)) x_o_Ex(i,j) = x_Ex;}
            else if(K == +1) {if(x_Ex >= x_o_Ex(i,j)) x_o_Ex(i,j) = x_Ex;}

            if(count == 1) x_o_Ex(i,j) = x_o(i,j) + K * dist/sin(phi);
            x_o_Ex(i+1,j) = x_o(i+1,j) + K * dist/sin(phi);
        }
    }

    return x_o_Ex;
}

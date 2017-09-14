#include <iostream>
#include <src/planners/highwayroadmap.h>

#define pi 3.1415926

highwayRoadmap::highwayRoadmap(SuperEllipse robot, double endpt[2][2], SuperEllipse* arena, SuperEllipse* obs, option opt){
    this->Robot = robot;
    this->Endpt = *endpt;
    this->Arena = arena;
    this->Obs = obs;

    this->infla = opt.infla;
    this->N_layers = opt.N_layers;
    this->N_dy = opt.N_dy;
    this->N_KCsample = opt.sampleNum;

    this->Lim = opt.Lim;
    this->isplot = opt.isplot;
    this->layerDist = opt.layerDist;
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
        cf_cell CFcell = this->rasterScan(bd);

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
    for(int i=0; i<sizeof(this->Arena); i++){
        bd.bd_s[i] = this->Arena[i].minkSum2D(this->Arena[i].a, robot_infla.a, this->Arena[i].num, -1);
    }
    for(int i=0; i<sizeof(this->Obs); i++){
        bd.bd_o[i] = this->Obs[i].minkSum2D(this->Obs[i].a, robot_infla.a, this->Obs[i].num, +1);
    }

    return bd;
}

cf_cell highwayRoadmap::rasterScan(boundary bd){

}

void highwayRoadmap::oneLayer(cf_cell CFcell){

    /*

this->vtxEdge.vertex.push_back({});
this->vtxEdge.edge.push_back(make_pair());

this->N_v_layers.pushback(sizeof(this->vtxEdge.vertex)/sizeof(this->vtxEdge.vertex[0]));*/
}

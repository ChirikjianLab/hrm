#include <iostream>
#include <src/planners/highwayroadmap.h>

#define pi 3.1415926

highwayRoadmap::highwayRoadmap(SuperEllipse robot, double endpt[2][2], SuperEllipse arena, SuperEllipse obs, option opt){
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

    for(int i=0; i<this->N_layers; i++){
        this->Robot.a[2] = dr*i;
        // boundary for obstacles and arenas
        boundary bd = this->boundaryGen();

        // collision-free cells, stored by ty, xL, xU, xM
        cf_cell CFcell = this->rasterScan(bd);

        // construct adjacency matrix for one layer
        AdjGraph oneGraph = this->oneLayer(CFcell);

        //this->Graph.adjMat.insert(this->Graph.adjMat.end(), oneGraph.adjMat.begin(), oneGraph.adjMat.end());
        //this->Graph.V.insert(this->Graph.V.end(), oneGraph.V.begin(), oneGraph.V.end());

        //this->N_v_layer[i] = sizeof(this->Graph.V);
    }
}

boundary highwayRoadmap::boundaryGen(){

}

cf_cell highwayRoadmap::rasterScan(boundary bd){

}

adjGraph highwayRoadmap::oneLayer(cf_cell CFcell){

}

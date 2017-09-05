#include <iostream>
#include <src/planners/highwayroadmap.h>

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

}

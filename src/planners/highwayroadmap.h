#ifndef HIGHWAYROADMAP_H
#define HIGHWAYROADMAP_H

#include <src/geometry/superellipse.h>

struct graph{
public:
    double V;
    int adjMat;
};

struct option{
    double infla;
    int N_layers, N_dy, sampleNum, layerDist, Lim[2];
    bool isplot;
};

class highwayRoadmap
{
    double infla;
    int N_layers, N_dy, N_v_layer, N_KCsample, layerDist, d12, I_start, I_goal, *Lim;
    bool isplot;
    double ang_r, polyVtx;

public:
    graph Graph;
    SuperEllipse Robot, Arena, Obs;
    double Cost, *Endpt;
    int Paths;

// functions
    highwayRoadmap(SuperEllipse robot, double endpt[2][2], SuperEllipse arena, SuperEllipse obs, option opt);
    void multiLayers();
};

#endif // HIGHWAYROADMAP_H

#ifndef HRM3D_MULTIBODY_H
#define HRM3D_MULTIBODY_H

#include <src/geometry/multibodytree3d.h>
#include <src/planners/highwayroadmap3d.h>

using namespace std;
#define pi 3.1415926

class hrm3d_multibody : public highwayRoadmap3D
{
private:
    vector<SuperQuadrics> mid;
    vector<cf_cell3D> mid_cell;
    vector<double> midVtx;
    double N_step = 3;

public:
    multibodytree3D RobotM;

public:
    hrm3d_multibody(multibodytree3D, vector< vector<double> >,
                    vector<SuperQuadrics>, vector<SuperQuadrics>, option3D);
    void plan();
    void buildRoadmap();
    boundary3D boundaryGen();
    void connectMultiLayer();

    vector<SuperQuadrics> tfe_multi(Quaterniond, Quaterniond);

    virtual ~hrm3d_multibody();

private:
    bool isCollisionFree(vector<double>, vector<double>);
    bool isPtInCFCell(cf_cell3D, vector<double>);
};

#endif // HRM3D_MULTIBODY_H

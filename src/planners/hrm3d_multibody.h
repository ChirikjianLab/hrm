#ifndef HRM3D_MULTIBODY_H
#define HRM3D_MULTIBODY_H

#include <src/geometry/multibodytree3d.h>
#include <src/planners/highwayroadmap3d.h>

using namespace std;

class hrm3d_multibody : public highwayRoadmap3D
{
public:
    multibodytree3D RobotM;

public:
    hrm3d_multibody(multibodytree3D, vector< vector<double> >,
                    vector<SuperQuadrics>, vector<SuperQuadrics>, option3D);
    void plan();
    void buildRoadmap();
    boundary3D boundaryGen();
    void connectMultiLayer();

    virtual ~hrm3d_multibody();
};

#endif // HRM3D_MULTIBODY_H

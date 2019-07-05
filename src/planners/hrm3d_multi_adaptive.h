#ifndef HRM3D_MULTI_ADAPTIVE_H
#define HRM3D_MULTI_ADAPTIVE_H

#include "src/planners/hrm3d_multibody.h"

class hrm3d_multi_adaptive : public hrm3d_multibody
{
public:
    hrm3d_multi_adaptive(multibodytree3D, vector< vector<double> >,
                         vector<SuperQuadrics>, vector<SuperQuadrics>, option3D);

    void planPath(double);
    virtual ~hrm3d_multi_adaptive();
};

#endif // HRM3D_MULTI_ADAPTIVE_H

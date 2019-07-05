#include "hrm3d_multi_adaptive.h"

hrm3d_multi_adaptive::hrm3d_multi_adaptive(multibodytree3D robot, vector< vector<double> > endPts,
                                           vector<SuperQuadrics> arena, vector<SuperQuadrics> obs, option3D opt)
    : hrm3d_multibody::hrm3d_multibody(robot, endPts, arena, obs, opt)
{}

void hrm3d_multi_adaptive::planPath(double timeLim){
    N_layers = 0;
    time::point start = time::now();
    {
        N_layers++;
        RobotM.Base.Shape.q = Quaterniond::UnitRandom();
        Robot.Shape.q = RobotM.Base.Shape.q;
        // boundary for obstacles and arenas
        boundary3D bd = boundaryGen();
        // collision-free cells, stored by tx, ty, zL, zU, zM
        cf_cell3D CFcell = sweepLineZ(bd.bd_s, bd.bd_o);
        // construct adjacency matrix for one layer
        connectOneLayer(CFcell);
        // Store the index of vertex in the current layer
        vtxId.push_back(N_v);

        // Connect multiple layers
        connectMultiLayer();

        // Search for a path
        search();

        planTime.totalTime = time::seconds(time::now() - start);
    } while(Paths.empty() && planTime.totalTime < timeLim);
}

hrm3d_multi_adaptive::~hrm3d_multi_adaptive(){}

#include "hrm3d_multibody.h"

hrm3d_multibody::hrm3d_multibody(multibodytree3D robot, vector< vector<double> > endpt,
                                 vector<SuperQuadrics> arena, vector<SuperQuadrics> obs, option3D opt) :
    highwayRoadmap3D::highwayRoadmap3D (robot.Base, endpt, arena, obs, opt){
    RobotM = robot;
}

void hrm3d_multibody::plan(){
    buildRoadmap();
    search();
}

// Build the roadmap for multi-rigid-body planning
void hrm3d_multibody::buildRoadmap(){
    // Samples from SO(3)
    sampleSO3();

    time::point start = time::now();

    for(size_t i=0; i<N_layers; i++){
        Robot.Shape.q = q_r[i];

        // boundary for obstacles and arenas
//        time::point start = time::now();
        boundary3D bd = boundaryGen();
//        cout << "Boundary: " << time::seconds(time::now() - start) << 's' << endl;

        // collision-free cells, stored by tx, ty, zL, zU, zM
//        start = time::now();
        cf_cell3D CFcell = sweepLineZ(bd.bd_s, bd.bd_o);
//        cout << "Sweep line: " << time::seconds(time::now() - start) << 's' << endl;

        // construct adjacency matrix for one layer
//        start = time::now();
        connectOneLayer(CFcell);
//        cout << "Connect within one layer: " << time::seconds(time::now() - start) << 's' << endl;

        // Store the index of vertex in the current layer
        vtxId.push_back(N_v);
    }
//    time::point start = time::now();
    connectMultiLayer();
//    cout << "Connect btw different layers: " << time::seconds(time::now() - start) << 's' << endl;

    planTime.buildTime = time::seconds(time::now() - start);
};

// Minkowski Boundary
boundary3D hrm3d_multibody::boundaryGen(){
    boundary3D bd;

    // Minkowski boundary points
    vector<MatrixXd> bd_aux;
    for(size_t i=0; i<N_s; i++){
        bd_aux = RobotM.minkSumSQ(Arena[i], 1);
        for(size_t j=0; j<bd_aux.size(); j++) bd.bd_s.push_back(bd_aux[j]);
        bd_aux.clear();
    }
    for(size_t i=0; i<N_o; i++){
        bd_aux = RobotM.minkSumSQ(Obs[i], 1);
        for(size_t j=0; j<bd_aux.size(); j++) bd.bd_o.push_back(bd_aux[j]);
        bd_aux.clear();
    }

    return bd;
}


void hrm3d_multibody::connectMultiLayer(){

}


hrm3d_multibody::~hrm3d_multibody(){};

#include "hrm3d_multi_adaptive.h"

hrm3d_multi_adaptive::hrm3d_multi_adaptive(multibodytree3D robot, vector< vector<double> > endPts,
                                           vector<SuperQuadrics> arena, vector<SuperQuadrics> obs, option3D opt)
    : hrm3d_multibody::hrm3d_multibody(robot, endPts, arena, obs, opt)
{}

void hrm3d_multi_adaptive::planPath(double timeLim){
    N_layers = 0;
    time::point start = time::now();
    do {
        N_layers++;
        q_r.push_back(Quaterniond::UnitRandom());
        RobotM.Base.Shape.q = q_r[N_layers-1];
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

void hrm3d_multi_adaptive::connectMultiLayer(){
    if(N_layers == 1) return;

    size_t n = vtxEdge.vertex.size(), start, n_1, n_2;
    vector<double> V1, V2;

    // Find vertex only in adjecent layers
    if(N_layers == 2){
        start = 0; n_1 = vtxId[0].layer;
        n_2 = vtxId[1].layer;
    } else {
        start = vtxId[N_layers-3].layer; n_1 = vtxId[N_layers-2].layer;
        n_2 = vtxId[N_layers-1].layer;
    }

    // Middle layer TFE and cell
    mid = tfe_multi(q_r[N_layers-2], q_r[N_layers-1]);
    for(size_t j=0; j<mid.size(); j++) mid_cell.push_back( midLayer(mid[j].Shape) );

    // Nearest vertex btw layers
    for(size_t m0=start; m0<n_1; m0++){
        V1 = vtxEdge.vertex[m0];
        for(size_t m1=n_1; m1<n_2; m1++){
            V2 = vtxEdge.vertex[m1];

            // Locate the nearest vertices
            if( fabs(V1[0]-V2[0]) > 1e-8 ||
                fabs(V1[1]-V2[1]) > 1e-8 ||
                fabs(V1[2]-V2[2]) > 1 ) continue;

            if( isCollisionFree(V1,V2) ){
                // Middle vertex: trans = V1; rot = V2;
                midVtx = {V1[0],V1[1],V1[2],V2[3],V2[4],V2[5],V2[6]};
                vtxEdge.vertex.push_back(midVtx);

                // Add new connections
                vtxEdge.edge.push_back(make_pair(m0, n));
                vtxEdge.weight.push_back( vector_dist(V1,midVtx) );
                vtxEdge.edge.push_back(make_pair(m1, n));
                vtxEdge.weight.push_back( vector_dist(V2,midVtx) );
                n++;
                break;
            }
        }
    }

    // Clear mid_cell and update the number of vertices
    mid_cell.clear();
    vtxId[N_layers-1].layer = vtxEdge.vertex.size();
}

hrm3d_multi_adaptive::~hrm3d_multi_adaptive(){};

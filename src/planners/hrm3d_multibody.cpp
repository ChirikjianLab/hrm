#include <iostream>
#include <fstream>
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
        RobotM.Base.Shape.q = q_r[i];
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

// Connect layers
void hrm3d_multibody::connectMultiLayer(){
    size_t n = vtxEdge.vertex.size(), n_1, n_12, n_2;
    size_t start = 0;

    for(size_t i=0; i<N_layers; i++){
        // Find vertex only in adjecent layers
        n_1 = vtxId[i].layer;

        // Construct the middle layer
        if(i == N_layers-1){
            n_12 = 0;
            n_2 = vtxId[0].layer;

            mid = tfe_multi(q_r[i], q_r[0]);

//            //
//            ofstream file_pose;
//            file_pose.open("robot_pose_mid.csv");
//            file_pose << q_r[i].w() << ',' << q_r[i].x() << ',' << q_r[i].y() << ',' << q_r[i].z() << endl <<
//                         q_r[0].w() << ',' << q_r[0].x() << ',' << q_r[0].y() << ',' << q_r[0].z() << endl;
//            file_pose.close();
//            //
        }
        else{
            n_12 = n_1;
            n_2 = vtxId[i+1].layer;

            mid = tfe_multi(q_r[i], q_r[i+1]);

//            //
//            ofstream file_pose;
//            file_pose.open("robot_pose_mid.csv");
//            file_pose << q_r[i].w() << ',' << q_r[i].x() << ',' << q_r[i].y() << ',' << q_r[i].z() << endl <<
//                         q_r[i+1].w() << ',' << q_r[i+1].x() << ',' << q_r[i+1].y() << ',' << q_r[i+1].z() << endl;
//            file_pose.close();
//            //
        }

//        //
//        ofstream file_mid;
//        file_mid.open("mid_3d.csv");
//        for(size_t i=0; i<mid.size(); i++) {
//            file_mid << mid[i].Shape.a[0] << ',' << mid[i].Shape.a[1] << ',' << mid[i].Shape.a[2] << ',' <<
//                        mid[i].Shape.pos[0] << ',' << mid[i].Shape.pos[1] << ',' << mid[i].Shape.pos[2] << ',' <<
//                        mid[i].Shape.q.w() << ',' << mid[i].Shape.q.x() << ',' << mid[i].Shape.q.y() << ',' <<
//                        mid[i].Shape.q.z() << endl;
//        }
//        file_mid.close();
//        //

        for(size_t j=0; j<mid.size(); j++) mid_cell.push_back( midLayer(mid[j].Shape) );

        // Nearest vertex btw layers
        for(size_t m0=start; m0<n_1; m0++){
            for(size_t m1=n_12; m1<n_2; m1++){
                if( fabs(vtxEdge.vertex[m0][0]-vtxEdge.vertex[m1][0]) <= 1e-8 &&
                    fabs(vtxEdge.vertex[m0][1]-vtxEdge.vertex[m1][1]) <= 1e-8 &&
                    fabs(vtxEdge.vertex[m0][2]-vtxEdge.vertex[m1][2]) <= 1 &&
                    isCollisionFree(vtxEdge.vertex[m0],vtxEdge.vertex[m1]) ){
                    // Middle vertex: trans = V1; rot = V2;
                    midVtx = {vtxEdge.vertex[m0][0],vtxEdge.vertex[m0][1],vtxEdge.vertex[m0][2],
                              vtxEdge.vertex[m1][3],vtxEdge.vertex[m1][4],vtxEdge.vertex[m1][5],vtxEdge.vertex[m1][6]};
                    vtxEdge.vertex.push_back(midVtx);

                    // Add new connections
                    vtxEdge.edge.push_back(make_pair(m0, n));
                    vtxEdge.weight.push_back( vector_dist(vtxEdge.vertex[m0],midVtx) );
                    vtxEdge.edge.push_back(make_pair(m1, n));
                    vtxEdge.weight.push_back( vector_dist(vtxEdge.vertex[m1],midVtx) );
                    n++;

                    // Continue from where it pauses
                    n_12 = m1;
                    break;
                }
            }
        }
        start = n_1;
    }
}


bool hrm3d_multibody::isCollisionFree(vector<double> V1, vector<double> V2){
    // Base: determine whether V1 is within any CF-Cell of midLayer
    if(!isPtInCFCell(mid_cell[0], V1)) return false;

    // Link i: determine Vi=V+RobotM.tf{1:3,4} within CF-cell of midLayer
    Matrix3d R1 = Quaterniond(V1[3],V1[4],V1[5],V1[6]).toRotationMatrix(),
             R2 = Quaterniond(V2[3],V2[4],V2[5],V2[6]).toRotationMatrix();
    AngleAxisd axang(R1.transpose() * R2), d_axang = axang;
    double dt = 1.0/N_step;
    Vector3d Vs;

    for(size_t i=0; i<RobotM.numLinks; i++){
        for(size_t j=0; j<N_step; j++){
            d_axang.angle() = j*dt*axang.angle();

            Vs = Vector3d({V1[0],V1[1],V1[2]}) +
                    R1*d_axang.toRotationMatrix()*RobotM.tf[i].block(0,3,3,1);

            if(!isPtInCFCell(mid_cell[i+1], {Vs[0],Vs[1],Vs[2]})) return false;
        }
    }

    return true;
}


// Point in collision-free cell
bool hrm3d_multibody::isPtInCFCell(cf_cell3D cell, vector<double> V){
    for(size_t i=1; i<cell.tx.size(); i++) {
        // Search for x-coord
        if(cell.tx[i] < V[0]) continue;

        // Search for the current plane
        cf_cellYZ cellX1 = cell.cellYZ[i], cellX2 = cell.cellYZ[i-1];
        for(size_t j=1; j<cellX1.ty.size();  j++){
            if(cellX1.ty[j] < V[1]) continue;

            // Within the range of current sweep line
            for(size_t k=0; k<cellX1.zM[j].size(); k++)
                if((V[2] < cellX1.zL[j][k]) || (V[2] > cellX1.zU[j][k])) return false;
            // Within the range of previous sweep line
            for(size_t k=0; k<cellX1.zM[j-1].size(); k++)
                if((V[2] < cellX1.zL[j-1][k]) || (V[2] > cellX1.zU[j-1][k])) return false;
        }

        // Search for the previou plane
        for(size_t j=1; j<cellX2.ty.size();  j++){
            if(cellX1.ty[j] < V[1]) continue;

            // Within the range of current sweep line
            for(size_t k=0; k<cellX2.zM[j].size(); k++)
                if((V[2] < cellX2.zL[j][k]) || (V[2] > cellX2.zU[j][k])) return false;
            // Within the range of previous sweep line
            for(size_t k=0; k<cellX2.zM[j-1].size(); k++)
                if((V[2] < cellX2.zL[j-1][k]) || (V[2] > cellX2.zU[j-1][k])) return false;
        }
    }

    // If within all the line segments
    return true;
}


// Multi-body Tightly-Fitted Ellipsoid
vector<SuperQuadrics> hrm3d_multibody::tfe_multi(Quaterniond q1, Quaterniond q2){
    multibodytree3D robot = RobotM;
    vector<SuperQuadrics> tfe_obj;

    // Two configurations
    Matrix3d R1 = q1.toRotationMatrix(), R2 = q2.toRotationMatrix();

    // Rotation axis and angle from R1 to R2
    AngleAxisd axang(R1.transpose() * R2);
    SuperQuadrics e_fitted;
    Matrix3d R_link;

    // Rotation angle > pi/2, fit a sphere
    if(fabs(axang.angle()) > pi/2){
        double ra = max(robot.Base.Shape.a[0], max(robot.Base.Shape.a[1], robot.Base.Shape.a[2]));
        e_fitted.Shape = {{ra,ra,ra}, {1,1}, {0,0,0}, Quaterniond::Identity(), 0};
        tfe_obj.push_back(e_fitted);

        for(size_t i=0; i<robot.numLinks; i++) {
            double rb = max(robot.Link[i].Shape.a[0], max(robot.Link[i].Shape.a[1], robot.Link[i].Shape.a[2]));
            e_fitted.Shape = {{rb,rb,rb}, {1,1}, {0,0,0}, Quaterniond::Identity(), 0};
            tfe_obj.push_back(e_fitted);
        }
    }
    // else fit a TFE
    else{
        double* ra; ra = robot.Base.Shape.a;
        e_fitted.Shape = tfe(ra, ra, q1, q2);
        tfe_obj.push_back(e_fitted);

        for(size_t i=0; i<robot.numLinks; i++) {
            double* rb; rb = robot.Link[i].Shape.a;
            R_link = robot.tf[i].block(0,0,3,3);
            e_fitted.Shape = tfe(rb, rb, Quaterniond(R1 * R_link), Quaterniond(R2 * R_link));
            tfe_obj.push_back(e_fitted);
        }
    }

    return tfe_obj;
}

hrm3d_multibody::~hrm3d_multibody(){};

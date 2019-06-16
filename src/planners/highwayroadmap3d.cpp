#include <iostream>
#include <list>
#include <random>
#include <src/planners/highwayroadmap3d.h>
#include <src/planners/interval.h>

#include <fstream>

#define pi 3.1415926

highwayRoadmap3D::highwayRoadmap3D(SuperQuadrics robot, vector< vector<double> > endpt,
                                   vector<SuperQuadrics> arena, vector<SuperQuadrics> obs, option3D opt){
    Robot = robot;
    Endpt = endpt;
    Arena = arena;
    Obs = obs;
    N_o = opt.N_o;
    N_s = opt.N_s;

    N_layers = opt.N_layers;
    N_dx = opt.N_dx;
    N_dy = opt.N_dy;
    Lim = opt.Lim;

    Cost = 0;

    cout << "Planning ..." << endl;
}

void highwayRoadmap3D::plan(){
    buildRoadmap();
    search();
}

void highwayRoadmap3D::buildRoadmap(){
    // Samples from SO(3)
    sampleSO3();
    // Compute mid-layer TFE
    for(size_t i=0; i<q_r.size(); i++){
        if(i == N_layers-1) mid.push_back( tfe(Robot.Shape.a, Robot.Shape.a, q_r[i], q_r[0]) );
        else mid.push_back( tfe(Robot.Shape.a, Robot.Shape.a, q_r[i], q_r[i+1]) );
    }

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
}


// ******************************************************************** //
// Generate Minkowski boundary
boundary3D highwayRoadmap3D::boundaryGen(){
    boundary3D bd;

    // calculate Minkowski boundary points
    for(size_t i=0; i<N_s; i++)
        bd.bd_s.push_back( Arena[i].minkSum3D(Robot.Shape,  -1) );
    for(size_t i=0; i<N_o; i++)
        bd.bd_o.push_back( Obs[i].minkSum3D(Robot.Shape, +1) );

    return bd;
}
// ******************************************************************** //


// ******************************************************************** //
// Generate collision-free vertices by Sweep Plane + Sweep Line process //

cf_cell3D highwayRoadmap3D::sweepLineZ(vector<MatrixXd> bd_s, vector<MatrixXd> bd_o){
    cf_cell3D CF_cell;
    intersectLineMesh3d lineMesh;
    vector<Vector3d> pts_o;

    MatrixXd z_s_L = MatrixXd::Constant(N_dy, N_s, -Lim[2]),
             z_s_R = MatrixXd::Constant(N_dy, N_s, Lim[2]),
             z_o_L = MatrixXd::Constant(N_dy, N_o, std::numeric_limits<double>::quiet_NaN()),
             z_o_R = MatrixXd::Constant(N_dy, N_o, std::numeric_limits<double>::quiet_NaN());

    // Find intersection points for each sweep line
    vector<double> tx(N_dx), ty(N_dy);
    double dx = 2*Lim[0]/(N_dx-1), dy = 2*Lim[1]/(N_dy-1);
    for(size_t i=0; i<N_dx; i++) tx[i] = -Lim[0] + i * dx;
    for(size_t i=0; i<N_dy; i++) ty[i] = -Lim[1] + i * dy;

    // Generate mesh for the boundaries
    vector<Mesh> P_s(N_s), P_o(N_o);
    for(size_t i=0; i<N_s; i++)
        P_s[i] = getMesh(bd_s[i], int(Arena[i].n));
    for(size_t i=0; i<N_o; i++)
        P_o[i] = getMesh(bd_o[i], int(Obs[i].n));

    // Find intersections along each sweep line
    for(size_t i=0; i<N_dx; i++){
//        time::point tstart = time::now();

        for(size_t j=0; j<N_dy; j++){
            VectorXd lineZ(6); lineZ << tx[i], ty[j], 0, 0, 0, 1;

            for(size_t m=0; m<N_s; m++){
                vector<Vector3d> pts_s = lineMesh.intersect(lineZ, P_s[m].vertices, P_s[m].faces);
                if(pts_s.empty()) continue;
                z_s_L(j,m) = min(pts_s[0](2), pts_s[1](2));
                z_s_R(j,m) = max(pts_s[0](2), pts_s[1](2));
            }
            for(size_t n=0; n<N_o; n++){
                pts_o = lineMesh.intersect(lineZ, P_o[n].vertices, P_o[n].faces);
                if(pts_o.empty()) continue;
                z_o_L(j,n) = min(pts_o[0](2), pts_o[1](2));
                z_o_R(j,n) = max(pts_o[0](2), pts_o[1](2));
            }
        }
//        cout << "Intersect: " << time::seconds(time::now() - tstart) << 's' << endl;

        // Store cell info
        CF_cell.tx.push_back(tx[i]);
        CF_cell.cellYZ.push_back( cfLine(ty, z_s_L, z_s_R, z_o_L, z_o_R) );

        z_s_L = MatrixXd::Constant(N_dy, N_s, -Lim[2]);
        z_s_R = MatrixXd::Constant(N_dy, N_s, Lim[2]);
        z_o_L = MatrixXd::Constant(N_dy, N_o, std::numeric_limits<double>::quiet_NaN());
        z_o_R = MatrixXd::Constant(N_dy, N_o, std::numeric_limits<double>::quiet_NaN());
    }
    return CF_cell;
}

// Sweep Line process at each sweep plane
cf_cellYZ highwayRoadmap3D::cfLine(vector<double> ty,
                                      MatrixXd x_s_L, MatrixXd x_s_R,
                                      MatrixXd x_o_L, MatrixXd x_o_R){
    cf_cellYZ cellYZ;

    interval op;
    vector<Interval> cf_seg[N_dy], obs_seg, arena_seg, obs_merge, arena_inter;
    vector<double> zL, zU, zM;

    // CF line segment for each ty
    for(size_t i=0; i<N_dy; i++){
        // Construct intervals at each sweep line
        for(size_t j=0; j<N_s; j++) if(!isnan(x_s_L(i,j)) && !isnan(x_s_R(i,j)))
            arena_seg.push_back({x_s_L(i,j), x_s_R(i,j)});
        for(size_t j=0; j<N_o; j++) if(!isnan(x_o_L(i,j)) && !isnan(x_o_R(i,j)))
            obs_seg.push_back({x_o_L(i,j), x_o_R(i,j)});

        // y-coord
        cellYZ.ty.push_back(ty[i]);

        // cf-intervals at each line
        obs_merge = op.Union(obs_seg);
        arena_inter = op.Intersect(arena_seg);
        cf_seg[i] = op.Complement(arena_inter,obs_merge);

        // x-coords
        for(size_t j=0; j<cf_seg[i].size(); j++){
            zL.push_back(cf_seg[i][j].s);
            zU.push_back(cf_seg[i][j].e);
            zM.push_back( (cf_seg[i][j].s+cf_seg[i][j].e)/2.0 );
        }
        cellYZ.zL.push_back(zL);
        cellYZ.zU.push_back(zU);
        cellYZ.zM.push_back(zM);

        // Clear memory
        arena_seg.clear();
        obs_seg.clear();
        zL.clear();
        zU.clear();
        zM.clear();
    }

    // Enhanced cell decomposition
    return enhanceDecomp(cellYZ);
}
// ******************************************************************** //


// ******************************************************************** //
// Connect vertices within one C-layer //
void highwayRoadmap3D::connectOneLayer(cf_cell3D cell){
    size_t I0=0, I1=0;

    N_v.line.clear();
    for(size_t i=0; i<cell.tx.size(); i++){
        connectOnePlane(cell.tx[i], cell.cellYZ[i]);
    }

    for(size_t i=0; i<cell.tx.size()-1; i++){
        for(size_t j=0; j<cell.cellYZ[i].ty.size(); j++){
            // Connect vertex btw adjacent planes, only connect with same ty
            for(size_t k0=0; k0<cell.cellYZ[i].zM[j].size(); k0++){
                I0 = N_v.line[i][j];
                for(size_t k1=0; k1<cell.cellYZ[i+1].zM[j].size(); k1++){
                    I1 = N_v.line[i+1][j];
                    if( ( (cell.cellYZ[i].zM[j][k0] > cell.cellYZ[i+1].zL[j][k1] &&
                           cell.cellYZ[i].zM[j][k0] < cell.cellYZ[i+1].zU[j][k1]) &&
                          (cell.cellYZ[i+1].zM[j][k1] > cell.cellYZ[i].zL[j][k0] &&
                           cell.cellYZ[i+1].zM[j][k1] < cell.cellYZ[i].zU[j][k0]) ) &&
                            ( (cell.cellYZ[i].zU[j][k0] > cell.cellYZ[i+1].zL[j][k1] &&
                               cell.cellYZ[i].zU[j][k0] < cell.cellYZ[i+1].zU[j][k1]) ||
                              (cell.cellYZ[i].zL[j][k0] > cell.cellYZ[i+1].zL[j][k1] &&
                               cell.cellYZ[i].zL[j][k0] < cell.cellYZ[i+1].zU[j][k1]) ||
                              (cell.cellYZ[i+1].zU[j][k1] > cell.cellYZ[i].zL[j][k0] &&
                               cell.cellYZ[i+1].zU[j][k1] < cell.cellYZ[i].zU[j][k0]) ||
                              (cell.cellYZ[i+1].zL[j][k1] > cell.cellYZ[i].zL[j][k0] &&
                               cell.cellYZ[i+1].zL[j][k1] < cell.cellYZ[i].zU[j][k0]) ) ){
                        vtxEdge.edge.push_back(make_pair(I0+k0, I1+k1));
                        vtxEdge.weight.push_back( vector_dist(vtxEdge.vertex[I0+k0],
                                                              vtxEdge.vertex[I1+k1]) );
                    }
                }
            }
        }
    }
}

void highwayRoadmap3D::connectOnePlane(double tx, cf_cellYZ CFcell){
    size_t N_0=0, N_1=0;

    N_v.plane.clear();
    for(size_t i=0; i<CFcell.ty.size(); i++){
        N_v.plane.push_back(vtxEdge.vertex.size());
        for(size_t j=0; j<CFcell.zM[i].size(); j++){
            // Construct a vector of vertex
            vtxEdge.vertex.push_back({tx, CFcell.ty[i], CFcell.zM[i][j],
                                      Robot.Shape.q.w(), Robot.Shape.q.x(),
                                      Robot.Shape.q.y(), Robot.Shape.q.z()});
        }
    }

    // Record index info
    N_v.line.push_back(N_v.plane);
    N_v.layer = vtxEdge.vertex.size();

    for(size_t i=0; i<CFcell.ty.size(); i++){
        N_0 = N_v.plane[i]; N_1 = N_v.plane[i+1];
        for(size_t j1=0; j1<CFcell.zM[i].size(); j1++){
            // Connect vertex within one sweep line
            if(j1 != CFcell.zM[i].size()-1){
                if(abs(CFcell.zU[i][j1] - CFcell.zL[i][j1+1]) < 1e-5){
                    vtxEdge.edge.push_back(make_pair(N_0+j1, N_0+j1+1));
                    vtxEdge.weight.push_back( vector_dist(vtxEdge.vertex[N_0+j1],vtxEdge.vertex[N_0+j1+1]) );
                }
            }
            // Connect vertex btw adjacent cells
            if(i != CFcell.ty.size()-1){
                for(size_t j2=0; j2<CFcell.zM[i+1].size(); j2++){
                    if( ( (CFcell.zM[i][j1] >= CFcell.zL[i+1][j2] && CFcell.zM[i][j1] <= CFcell.zU[i+1][j2]) &&
                          (CFcell.zM[i+1][j2] >= CFcell.zL[i][j1] && CFcell.zM[i+1][j2] <= CFcell.zU[i][j1]) ) &&
                        ( (CFcell.zU[i][j1] >= CFcell.zL[i+1][j2] && CFcell.zU[i][j1] <= CFcell.zU[i+1][j2]) ||
                          (CFcell.zL[i][j1] >= CFcell.zL[i+1][j2] && CFcell.zL[i][j1] <= CFcell.zU[i+1][j2]) ||
                          (CFcell.zU[i+1][j2] >= CFcell.zL[i][j1] && CFcell.zU[i+1][j2] <= CFcell.zU[i][j1]) ||
                          (CFcell.zL[i+1][j2] >= CFcell.zL[i][j1] && CFcell.zL[i+1][j2] <= CFcell.zU[i][j1]) ) ){
                        vtxEdge.edge.push_back(make_pair(N_0+j1, N_1+j2));
                        vtxEdge.weight.push_back( vector_dist(vtxEdge.vertex[N_0+j1],vtxEdge.vertex[N_1+j2]) );
                    }
                }
            }
        }
    }
}
// ******************************************************************** //


// ******************************************************************** //
// Connect vertices within adjacent C-layers //
void highwayRoadmap3D::connectMultiLayer(){
    size_t n = vtxEdge.vertex.size(), n_1, n_12, n_2;
    size_t start = 0;

    for(size_t i=0; i<N_layers; i++){
        // Find vertex only in adjecent layers
        n_1 = vtxId[i].layer;

        // Construct the middle layer
        if(i == N_layers-1){
            n_12 = 0;
            n_2 = vtxId[0].layer;
        }
        else{
            n_12 = n_1;
            n_2 = vtxId[i+1].layer;
        }
        midLayer(mid[i]);

        // Nearest vertex btw layers
        for(size_t m0=start; m0<n_1; m0++){
            for(size_t m1=n_12; m1<n_2; m1++){
                if( fabs(vtxEdge.vertex[m0][0]-vtxEdge.vertex[m1][0]) <= 1e-8 &&
                    fabs(vtxEdge.vertex[m0][1]-vtxEdge.vertex[m1][1]) <= 1e-8 &&
                    isPtinCFLine(vtxEdge.vertex[m0],vtxEdge.vertex[m1]) ){
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
// ******************************************************************** //

void highwayRoadmap3D::search(){
    time::point start = time::now();

    Vertex idx_s, idx_g, num;

    // Construct the roadmap
    size_t num_vtx = vtxEdge.vertex.size();
    AdjGraph g(num_vtx);

    for(size_t i=0; i<vtxEdge.edge.size(); i++)
        add_edge(size_t(vtxEdge.edge[i].first), size_t(vtxEdge.edge[i].second),
                 Weight(vtxEdge.weight[i]), g);

    // Locate the nearest vertex for start and goal in the roadmap
    idx_s = find_cell(Endpt[0]);
    idx_g = find_cell(Endpt[1]);

    // Search for shortest path
    std::vector<Vertex> p(num_vertices(g));
    std::vector<double> d(num_vertices(g));
    astar_search( g, idx_s, [this, idx_g](Vertex v){ return vector_dist(vtxEdge.vertex[v], vtxEdge.vertex[idx_g]); },
                  predecessor_map(make_iterator_property_map(p.begin(), get(vertex_index, g))).
                  distance_map(make_iterator_property_map(d.begin(), get(vertex_index, g))) );

    // Record path and cost
    num = 0;
    Paths.push_back(int(idx_g));
    while(Paths[num] != int(idx_s) && num <= num_vtx){
        Paths.push_back( int(p[ size_t(Paths[num]) ]) );
        Cost += d[ size_t(Paths[num]) ];
        num++;
    }
    if(num == num_vtx+1) Paths.clear();
    if(Paths.size() > 0) flag = true;

    planTime.searchTime = time::seconds(time::now() - start);
}


/************************************************************************************************/
/*********************************** Private Functions ******************************************/
/************************************************************************************************/

// Make sure all connections between vertexes are within one convex cell
cf_cellYZ highwayRoadmap3D::enhanceDecomp(cf_cellYZ cell){
    cf_cellYZ cell_new = cell;

    for(size_t i=0; i<cell.ty.size()-1; i++){
        for(size_t j1=0; j1<cell.zM[i].size(); j1++){
            for(size_t j2=0; j2<cell.zM[i+1].size(); j2++){
                if(cell_new.zM[i][j1] < cell_new.zL[i+1][j2] && cell_new.zU[i][j1] >= cell_new.zL[i+1][j2]){
                    cell_new.zU[i].push_back(cell_new.zL[i+1][j2]);
                    cell_new.zL[i].push_back(cell_new.zL[i+1][j2]);
                    cell_new.zM[i].push_back(cell_new.zL[i+1][j2]);
                }
                else if(cell_new.zM[i][j1] > cell_new.zU[i+1][j2] && cell_new.zL[i][j1] <= cell_new.zU[i+1][j2]){
                    cell_new.zU[i].push_back(cell_new.zU[i+1][j2]);
                    cell_new.zL[i].push_back(cell_new.zU[i+1][j2]);
                    cell_new.zM[i].push_back(cell_new.zU[i+1][j2]);
                }

                if(cell_new.zM[i+1][j2] < cell_new.zL[i][j1] && cell_new.zU[i+1][j2] >= cell_new.zL[i][j1]){
                    cell_new.zU[i+1].push_back(cell_new.zL[i][j1]);
                    cell_new.zL[i+1].push_back(cell_new.zL[i][j1]);
                    cell_new.zM[i+1].push_back(cell_new.zL[i][j1]);
                }
                else if(cell_new.zM[i+1][j2] > cell_new.zU[i][j1] && cell_new.zL[i+1][j2] <= cell_new.zU[i][j1]){
                    cell_new.zU[i+1].push_back(cell_new.zU[i][j1]);
                    cell_new.zL[i+1].push_back(cell_new.zU[i][j1]);
                    cell_new.zM[i+1].push_back(cell_new.zU[i][j1]);
                }
            }
        }

        sort(cell_new.zL[i].begin(), cell_new.zL[i].end(), [](double a, double b){return a < b;});
        sort(cell_new.zU[i].begin(), cell_new.zU[i].end(), [](double a, double b){return a < b;});
        sort(cell_new.zM[i].begin(), cell_new.zM[i].end(), [](double a, double b){return a < b;});
    }

    return cell_new;
}

// Connect vertexes among different layers
void highwayRoadmap3D::midLayer(SuperQuadrics::shape Ec){
    boundary3D bd;
    // calculate Minkowski boundary points
    for(size_t i=0; i<N_s; i++) bd.bd_s.push_back(Arena[i].minkSum3D(Ec,-1));
    for(size_t i=0; i<N_o; i++) bd.bd_o.push_back(Obs[i].minkSum3D(Ec,+1));

    mid_cell = sweepLineZ(bd.bd_s, bd.bd_o);
}

bool highwayRoadmap3D::isPtinCFLine(vector<double> V1, vector<double> V2){
    for(size_t i=0; i<mid_cell.tx.size(); i++){
        if(fabs(mid_cell.tx[i]-V1[0])>1e-8) continue;

        for(size_t j=0; j<mid_cell.cellYZ[i].ty.size(); j++) {
            if(fabs(mid_cell.cellYZ[i].ty[j]-V1[1])>1e-8) continue;

            for(size_t k=0; k<mid_cell.cellYZ[i].zM[j].size(); k++)
                if((V1[2] >= mid_cell.cellYZ[i].zL[j][k]) &&
                   (V1[2] <= mid_cell.cellYZ[i].zU[j][k]) &&
                   (V2[2] >= mid_cell.cellYZ[i].zL[j][k]) &&
                   (V2[2] <= mid_cell.cellYZ[i].zU[j][k])) return 1;
        }
    }
    return 0;
}

// Sampled rotations on SO(3), return a list of Quaternions
void highwayRoadmap3D::sampleSO3(){
    if(Robot.Shape.q_sample.empty())
        // Uniform random samples for Quaternions
        for(size_t i=0; i<N_layers; i++) q_r.push_back(Quaterniond::UnitRandom());
    else{
        // Pre-defined samples of Quaternions
        N_layers = Robot.Shape.q_sample.size();
        q_r = Robot.Shape.q_sample;
    }
}

// Find the cell that an arbitrary vertex locates, and find the closest roadmap vertex
unsigned int highwayRoadmap3D::find_cell(vector<double> v){
    double d_min, d;
    unsigned int idx = 0;

    d_min = vector_dist({v[0],v[1],v[2]},{vtxEdge.vertex[0][0],vtxEdge.vertex[0][1],vtxEdge.vertex[0][2]});
    for(unsigned int i=0; i<vtxEdge.vertex.size(); i++){
        d = vector_dist({v[0],v[1],v[2]},{vtxEdge.vertex[i][0],vtxEdge.vertex[i][1],vtxEdge.vertex[i][2]});
        if(d < d_min){
            d_min = d;
            idx = i;
        }
    }
    return idx;
}

// Compute Euclidean distance between two vectors
double highwayRoadmap3D::vector_dist(vector<double> v1, vector<double> v2){
    vector<double> diff;
    for(size_t i=0; i<v1.size(); i++) diff.push_back(v1[i]-v2[i]);
    return sqrt( inner_product(diff.begin(), diff.end(), diff.begin(), 0.0) );
}

// Generate mesh info from a ordered vertex list, i.e. "surf"
Mesh highwayRoadmap3D::getMesh(MatrixXd bd, int n){
    Mesh M;
    int Num = (n-1)*(n-1);
    ArrayXd q((n-1)*(n-1));
    for(int i=0; i<n-1; i++) q.segment(i*(n-1),(n-1)) = ArrayXd::LinSpaced(n-1,i*n,(i+1)*n-2);

    M.vertices = bd;

    M.faces = MatrixXd::Zero(2*Num,3);
    M.faces.block(0,0,Num,1) = q;
    M.faces.block(0,1,Num,1) = q + n;
    M.faces.block(0,2,Num,1) = q + n + 1;
    M.faces.block(Num,0,Num,1) = q;
    M.faces.block(Num,1,Num,1) = q + 1;
    M.faces.block(Num,2,Num,1) = q + n + 1;

    return M;
}

// Compute the Tightly-Fitted Ellipsoid enclosing two ellipsoids with the same center
SuperQuadrics::shape highwayRoadmap3D::tfe(double* a, double* b, Quaterniond q_a, Quaterniond q_b){
    Matrix3d Ra = q_a.toRotationMatrix(),
             Rb = q_b.toRotationMatrix();

    double r = fmin(b[0],fmin(b[1],b[2]));
    DiagonalMatrix<double, 3> diag, diag_a, diag_c;
    diag.diagonal() = Array3d(r/b[0],r/b[1],r/b[2]);
    diag_a.diagonal() = Array3d(pow(a[0],-2),pow(a[1],-2),pow(a[2],-2));

    // Shrinking affine transformation
    Matrix3d T = Rb * diag * Rb.transpose();

    // In shrunk space, fit ellipsoid Cp to sphere Bp and ellipsoid Ap
    Matrix3d Ap = T.inverse() * (Ra * diag_a * Ra.transpose()) * T.inverse();
    JacobiSVD<Matrix3d> svd(Ap, ComputeFullU | ComputeFullV);
    Array3d a_p = svd.singularValues().array().pow(-0.5);
    Array3d c_p = {max(a_p(0),r), max(a_p(1),r), max(a_p(2),r)};

    // Stretch back
    diag_c.diagonal() = c_p.pow(-2);
    Matrix3d C = T * svd.matrixU() * diag_c * svd.matrixU().transpose() * T;
    svd.compute(C);
    Quaterniond q_c(svd.matrixU());
    Array3d c = svd.singularValues().array().pow(-0.5);

    return SuperQuadrics::shape({{c(0),c(1),c(2)}, {1,1}, {0,0,0}, {q_c.w(),q_c.x(),q_c.y(),q_c.z()}, 0});
}

highwayRoadmap3D::~highwayRoadmap3D(){};

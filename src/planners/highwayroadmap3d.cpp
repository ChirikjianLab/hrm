#include <iostream>
#include <list>
#include <random>
#include <src/planners/highwayroadmap3d.h>
#include <src/planners/interval.h>
#include <src/geometry/ptinpoly.h>

#include <fstream>

#define pi 3.1415926

highwayRoadmap3D::highwayRoadmap3D(vector<SuperQuadrics> robot, vector< vector<double> > endpt,
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
    time::point start = time::now();
    buildRoadmap();
    planTime.buildTime = time::seconds(time::now() - start);

    start = time::now();
    search();
    planTime.searchTime = time::seconds(time::now() - start);
}

void highwayRoadmap3D::buildRoadmap(){
    graph Graph;

    // Samples from SO(3)
    sampleSO3();

    for(size_t i=0; i<N_layers; i++){
        for(size_t j=0; j<q_r[i].size(); j++)
            Robot[0].Shape.q[j] = q_r[i][j];

        // boundary for obstacles and arenas
        boundary3D bd = boundaryGen();

        // collision-free cells, stored by tx, ty, zL, zU, zM
        cf_cell3D CFcell = sweepLineZ(bd.bd_s, bd.bd_o);

        // construct adjacency matrix for one layer
        connectOneLayer(CFcell);

        // Store the index of vertex in the current layer
        vtxId.push_back(N_v);
    }
    connectMultiLayer();
}


// ******************************************************************** //
// Generate Minkowski boundary
boundary3D highwayRoadmap3D::boundaryGen(){
    boundary3D bd;

    for(size_t num_r = 0; num_r < Robot.size(); num_r++){
        // calculate Minkowski boundary points
        for(size_t i=0; i<N_s; i++)
            bd.bd_s.push_back( Arena[i].minkSum3D(Robot[num_r].Shape,  -1) );
        for(size_t i=0; i<N_o; i++)
            bd.bd_o.push_back( Obs[i].minkSum3D(Robot[num_r].Shape, +1) );
    }

    return bd;
}
// ******************************************************************** //


// ******************************************************************** //
// Generate collision-free vertices by Sweep Plane + Sweep Line process //

cf_cell3D highwayRoadmap3D::sweepLineZ(vector<MatrixXd> bd_s, vector<MatrixXd> bd_o){
    cf_cell3D CF_cell;
    intersectLineMesh3d lineMesh;

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
        MatrixXd z_s_L(N_dy, N_s), z_s_R(N_dy, N_s),
                 z_o_L(N_dy, N_o), z_o_R(N_dy, N_o);

        for(size_t j=0; j<N_dy; j++){
            VectorXd lineZ(6); lineZ << tx[i], ty[j], 0, 0, 0, 1;

            for(size_t m=0; m<N_s; m++){
                vector<Vector3d> pts_s = lineMesh.intersect(lineZ, P_s[m].vertices, P_s[m].faces);
                if(pts_s.empty()) continue;
                z_s_L(j,m) = min(pts_s[0](2), pts_s[1](2));
                z_s_R(j,m) = max(pts_s[0](2), pts_s[1](2));
            }
            for(size_t n=0; n<N_o; n++){
                vector<Vector3d> pts_o = lineMesh.intersect(lineZ, P_o[n].vertices, P_o[n].faces);
                if(pts_o.empty()) continue;
                z_o_L(j,n) = min(pts_o[0](2), pts_o[1](2));
                z_o_R(j,n) = max(pts_o[0](2), pts_o[1](2));
            }
        }

        // Store cell info
        CF_cell.tx.push_back(tx[i]);
        CF_cell.cellYZ.push_back( cfLine(ty, z_s_L, z_s_R, z_o_L, z_o_R) );
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
                                      Robot[0].Shape.q[0], Robot[0].Shape.q[1],
                                      Robot[0].Shape.q[2], Robot[0].Shape.q[3]});
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
                    if( ( (CFcell.zM[i][j1] > CFcell.zL[i+1][j2] && CFcell.zM[i][j1] < CFcell.zU[i+1][j2]) &&
                          (CFcell.zM[i+1][j2] > CFcell.zL[i][j1] && CFcell.zM[i+1][j2] < CFcell.zU[i][j1]) ) &&
                        ( (CFcell.zU[i][j1] > CFcell.zL[i+1][j2] && CFcell.zU[i][j1] < CFcell.zU[i+1][j2]) ||
                          (CFcell.zL[i][j1] > CFcell.zL[i+1][j2] && CFcell.zL[i][j1] < CFcell.zU[i+1][j2]) ||
                          (CFcell.zU[i+1][j2] > CFcell.zL[i][j1] && CFcell.zU[i+1][j2] < CFcell.zU[i][j1]) ||
                          (CFcell.zL[i+1][j2] > CFcell.zL[i][j1] && CFcell.zL[i+1][j2] < CFcell.zU[i][j1]) ) ){
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
    size_t n, n_11, n_12, n_2;
    double d;
    size_t start = 0;
    vector<double> midVtx;
    vector<double> v1, v2;

    n = vtxEdge.vertex.size();

    for(size_t i=0; i<N_layers; i++){
        // Find vertex only in adjecent layers
        n_11 = vtxId[i].layer;
        n_2 = vtxId[i+1].layer;

        // Nearest vertex btw layers
        for(size_t m=start; m<n_11; m++){
            v1 = vtxEdge.vertex[m];
            n_12 = n_11;
            if(i == N_layers-1){
                n_2 = vtxId[0].layer;
                v1[2] = 0.0;
                n_12 = 0;
            }
            for(size_t m2=n_12; m2<n_2; m2++){
                v2 = vtxEdge.vertex[m2];
                if(vector_dist(v1,v2) <= 0.1) midVtx = addMidVtx(v1,v2);
                if(!midVtx.empty()){
                    vtxEdge.vertex.push_back(midVtx);

                    vtxEdge.edge.push_back(make_pair(m, n));
                    vtxEdge.weight.push_back( vector_dist(v1,midVtx) );
                    vtxEdge.edge.push_back(make_pair(m2, n));
                    vtxEdge.weight.push_back( vector_dist(v2,midVtx) );
                    n++;
                    break;
                }
            }
            midVtx.clear();
        }
        start = n_11;
    }
}
// ******************************************************************** //

void highwayRoadmap3D::search(){
    unsigned int idx_s, idx_g, num;

    // Construct the roadmap
    int num_vtx = vtxEdge.vertex.size();
    AdjGraph g(num_vtx);

    for(size_t i=0; i<vtxEdge.edge.size(); i++)
        add_edge(vtxEdge.edge[i].first, vtxEdge.edge[i].second, Weight(vtxEdge.weight[i]) ,g);

    // Locate the nearest vertex for start and goal in the roadmap
    idx_s = find_cell(Endpt[0]);
    idx_g = find_cell(Endpt[1]);

    // Search for shortest path
    std::vector<Vertex> p(num_vertices(g));
    std::vector<double> d(num_vertices(g));

    dijkstra_shortest_paths(g, idx_g,
                            predecessor_map(make_iterator_property_map(p.begin(), get(vertex_index, g))).
                            distance_map(make_iterator_property_map(d.begin(), get(vertex_index, g))));
//    astar_search( g, idx_g, distance_heuristic<AdjGraph, int>(idx_g, g),
//                  predecessor_map(make_iterator_property_map(p.begin(), get(vertex_index, g))).
//                  distance_map(make_iterator_property_map(d.begin(), get(vertex_index, g))) );

    // Record path and cost
    num = 0;
    Paths.push_back(idx_s);
    while(Paths[num] != idx_g && num <= num_vtx){
        Paths.push_back(p[Paths[num]]);
        Cost += d[Paths[num]];
        num++;
    }
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

// Connect vertexes among different layers, and add a middle vertex to the roadmap
vector<double> highwayRoadmap3D::addMidVtx(vector<double> vtx1, vector<double> vtx2){
    vector<double> midVtx, pt, pt1, pt2;
//    ptInPoly3D polyTest;
//    bool flag;
//    midVtx.clear();

//    for(size_t iter = 0; iter<N_KCsample; iter++){
//        pt.clear(); pt1.clear(); pt2.clear();
//        for(size_t i=0; i<vtx1.size(); i++){
//            pt.push_back( (rand() * (vtx1[i]-vtx2[i]))/RAND_MAX + vtx2[i] );
//            pt1.push_back(pt[i] - vtx1[i]);
//            pt2.push_back(pt[i] - vtx2[i]);
//        }
//        flag = polyTest.isPtInPoly3D(polyVtx, pt1);
//        if(flag){
//            flag = polyTest.isPtInPoly3D(polyVtx, pt2);
//            if(flag){
//                midVtx = pt;
//                return midVtx;
//            }
//        }
//    }

    return midVtx;
}

// Uniform random sampling on SO(3)
void highwayRoadmap3D::sampleSO3(){
    vector<double> u(3);

    // Uniform random samples for Quaternions
    for(size_t i=0; i<N_layers; i++){
        u = {rand()*0.5/RAND_MAX, rand()*0.5/RAND_MAX, rand()*0.5/RAND_MAX};
        this->q_r.push_back({sqrt(1-u[0])*sin(2*pi*u[1]),
                             sqrt(1-u[0])*cos(2*pi*u[1]),
                             sqrt(u[0])*sin(2*pi*u[2]),
                             sqrt(u[0])*cos(2*pi*u[2])});
    }
}

// Find the cell that an arbitrary vertex locates, and find the closest roadmap vertex
unsigned int highwayRoadmap3D::find_cell(vector<double> v){
    double d_min, d;
    unsigned int idx = 0;

    d_min = vector_dist(v,vtxEdge.vertex[0]);
    for(unsigned int i=0; i<vtxEdge.vertex.size(); i++){
        d = vector_dist(v,vtxEdge.vertex[i]);
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

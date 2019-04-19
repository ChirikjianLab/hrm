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
        cf_cell3D CFcell = sweepPlane(bd.bd_s, bd.bd_o);

        // construct adjacency matrix for one layer
        connectOneLayer(CFcell);

        // Store the number of vertex before the current layer
        N_v_layer.push_back(vtxEdge.vertex.size());
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
        CF_cell.cellYZ.push_back( sweepLine(ty, z_s_L, z_s_R, z_o_L, z_o_R) );
    }

    return CF_cell;
}

cf_cell3D highwayRoadmap3D::sweepPlane(vector<MatrixXd> bd_s, vector<MatrixXd> bd_o){
    cf_cell3D cell;

    boundary3D::sepZ z_bd_s, z_bd_o;

    MatrixXd bd_s_L[N_s], bd_s_R[N_s], bd_o_L[N_o], bd_o_R[N_o];
    MatrixXd x_s_L(N_dy, N_s), x_s_R(N_dy, N_s);
    MatrixXd x_o_L(N_dy, N_o), x_o_R(N_dy, N_o);

    // Find closest points for each raster scan line
    vector<double> tx(N_dx), ty(N_dy);
    double dx = 2*Lim[0]/(N_dx-1), dy = 2*Lim[1]/(N_dy-1);
    for(size_t i=0; i<N_dx; i++) tx.push_back(-Lim[0] + i * dx);
    for(size_t i=0; i<N_dy; i++) ty.push_back(-Lim[1] + i * dy);

    for(size_t i=0; i<N_dx; i++){
        // separated z-coord
        z_bd_s = closestPt(bd_s, tx[i], ty);
        z_bd_o = closestPt(bd_o, tx[i], ty);

        // Store x-coordinate of each sweep plane
        cell.tx[i] = tx[i];
        // Store y,z-coord into the cell
        cell.cellYZ[i] = sweepLine(ty,
                                   z_bd_s.z_L, z_bd_s.z_R,
                                   z_bd_o.z_L, z_bd_o.z_R);

    //    ofstream file_obs;
    //    file_obs.open("bd_obs.csv");
    //    file_obs << x_o_L << "\n";
    //    file_obs << x_o_R << "\n";
    //    file_obs.close();

    //    // Enlarge the obstacle to form convex CF cells
    //    x_o_L = boundaryEnlarge(bd_o_L, x_o_L, ty, -1);
    //    x_o_R = boundaryEnlarge(bd_o_R, x_o_R, ty, +1);

    //    // write to .csv file
    //    ofstream file_ty;
    //    file_ty.open("bd_ty.csv");
    //    for(int i=0; i<N_dy; i++) file_ty << ty[i] << "\n";
    //    file_ty.close();

    //    file_obs.open("bd_obs_ex.csv");
    //    file_obs << x_o_L << "\n";
    //    file_obs << x_o_R << "\n";
    //    file_obs.close();

    //    ofstream file_arena;
    //    file_arena.open("bd_arena.csv");
    //    file_arena << x_s_L << "\n";
    //    file_arena << x_s_R << "\n";
    //    file_arena.close();

    }

    return cell;
}

// Sweep Line process at each sweep plane
cf_cellYZ highwayRoadmap3D::sweepLine(vector<double> ty,
                                      MatrixXd x_s_L, MatrixXd x_s_R,
                                      MatrixXd x_o_L, MatrixXd x_o_R){
    cf_cellYZ cellYZ;

    interval op;
    vector<Interval> cf_seg[N_dy], obs_seg, arena_seg, obs_merge, arena_inter;
    vector<double> xL, xU, xM;

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
            xL.push_back(cf_seg[i][j].s);
            xU.push_back(cf_seg[i][j].e);
            xM.push_back( (cf_seg[i][j].s+cf_seg[i][j].e)/2.0 );
        }
        cellYZ.xL.push_back(xL);
        cellYZ.xU.push_back(xU);
        cellYZ.xM.push_back(xM);

        // Clear memory
        arena_seg.clear();
        obs_seg.clear();
        xL.clear();
        xU.clear();
        xM.clear();
    }

    // Enhanced cell decomposition
    return enhanceDecomp(cellYZ);
}
// ******************************************************************** //


// ******************************************************************** //
// Connect vertices within one C-layer //
void highwayRoadmap3D::connectOneLayer(cf_cell3D cell){
    vector<unsigned int> N_v_plane;
    unsigned int N_0=0, N_1=0;

    for(size_t i=0; i<cell.tx.size(); i++){
        N_v_plane.push_back(vtxEdge.vertex.size());
        connectOnePlane(cell.tx[i], cell.cellYZ[i]);
    }
    for(size_t i=0; i<cell.tx.size()-1; i++){
        N_0 = N_v_plane[i]; N_1 = N_v_plane[i+1];
        for(size_t j1=0; j1<cell.cellYZ[i].ty.size(); j1++){
            // Connect vertex btw adjacent planes
            for(size_t j2=0; j2<cell.cellYZ[i+1].ty.size(); j2++){
                if( ( (cell.cellYZ[i].xM[j1] > cell.cellYZ[i+1].xL[j2] && cell.cellYZ[i].xM[j1] < cell.cellYZ[i+1].xU[j2]) ||
                      (cell.cellYZ[i+1].xM[j2] > cell.cellYZ[i].xL[j1] && cell.cellYZ[i+1].xM[j2] < cell.cellYZ[i].xU[j1]) ) &&
                    ( (cell.cellYZ[i].xU[j1] > cell.cellYZ[i+1].xL[j2] && cell.cellYZ[i].xU[j1] < cell.cellYZ[i+1].xU[j2]) ||
                      (cell.cellYZ[i].xL[j1] > cell.cellYZ[i+1].xL[j2] && cell.cellYZ[i].xL[j1] < cell.cellYZ[i+1].xU[j2]) ||
                      (cell.cellYZ[i+1].xU[j2] > cell.cellYZ[i].xL[j1] && cell.cellYZ[i+1].xU[j2] < cell.cellYZ[i].xU[j1]) ||
                      (cell.cellYZ[i+1].xL[j2] > cell.cellYZ[i].xL[j1] && cell.cellYZ[i+1].xL[j2] < cell.cellYZ[i].xU[j1]) ) ){
                    vtxEdge.edge.push_back(make_pair(N_0+j1, N_1+j2));
                    vtxEdge.weight.push_back( vector_dist(vtxEdge.vertex[N_0+j1],vtxEdge.vertex[N_1+j2]) );
                }
            }
        }
    }
}

void highwayRoadmap3D::connectOnePlane(double tz, cf_cellYZ CFcell){
    vector<unsigned int> N_v_line;
    unsigned int N_0=0, N_1=0;

    for(size_t i=0; i<CFcell.ty.size(); i++){
        N_v_line.push_back(vtxEdge.vertex.size());

        for(size_t j=0; j<CFcell.xM[i].size(); j++){
            // Construct a vector of vertex
            vtxEdge.vertex.push_back({CFcell.xM[i][j], CFcell.ty[i], tz,
                                      Robot[0].Shape.q[0], Robot[0].Shape.q[1],
                                      Robot[0].Shape.q[2], Robot[0].Shape.q[3]});
        }
    }
    for(size_t i=0; i<CFcell.ty.size(); i++){
        N_0 = N_v_line[i]; N_1 = N_v_line[i+1];
        for(size_t j1=0; j1<CFcell.xM[i].size(); j1++){
            // Connect vertex within one sweep line
            if(j1 != CFcell.xM[i].size()-1){
                if(abs(CFcell.xU[i][j1] - CFcell.xL[i][j1+1]) < 1e-5){
                    vtxEdge.edge.push_back(make_pair(N_0+j1, N_0+j1+1));
                    vtxEdge.weight.push_back( vector_dist(vtxEdge.vertex[N_0+j1],vtxEdge.vertex[N_0+j1+1]) );
                }
            }
            // Connect vertex btw adjacent cells
            if(i != CFcell.ty.size()-1){
                for(size_t j2=0; j2<CFcell.xM[i+1].size(); j2++){
                    if( ( (CFcell.xM[i][j1] > CFcell.xL[i+1][j2] && CFcell.xM[i][j1] < CFcell.xU[i+1][j2]) ||
                          (CFcell.xM[i+1][j2] > CFcell.xL[i][j1] && CFcell.xM[i+1][j2] < CFcell.xU[i][j1]) ) &&
                        ( (CFcell.xU[i][j1] > CFcell.xL[i+1][j2] && CFcell.xU[i][j1] < CFcell.xU[i+1][j2]) ||
                          (CFcell.xL[i][j1] > CFcell.xL[i+1][j2] && CFcell.xL[i][j1] < CFcell.xU[i+1][j2]) ||
                          (CFcell.xU[i+1][j2] > CFcell.xL[i][j1] && CFcell.xU[i+1][j2] < CFcell.xU[i][j1]) ||
                          (CFcell.xL[i+1][j2] > CFcell.xL[i][j1] && CFcell.xL[i+1][j2] < CFcell.xU[i][j1]) ) ){
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
        n_11 = N_v_layer[i];
        n_2 = N_v_layer[i+1];

        // Nearest vertex btw layers
        for(size_t m=start; m<n_11; m++){
            v1 = vtxEdge.vertex[m];
            n_12 = n_11;
            if(i == N_layers-1){
                n_2 = N_v_layer[0];
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
// For a given curve, separate its boundary into two parts
boundary3D::sepBd highwayRoadmap3D::separateBoundary(MatrixXd bd){
    const int half_num = Arena[0].num/2;
    MatrixXd::Index I_max_y, I_min_y;
    int I_start_y;
    MatrixXd P_bd_L, P_bd_R;
    double max_y, min_y;
    boundary3D::sepBd P_bd;

    // Find separating point
    max_y = bd.row(2).maxCoeff(&I_max_y);
    min_y = bd.row(2).minCoeff(&I_min_y);
    I_start_y = min(I_max_y, I_min_y);
    I_start_y = min(I_start_y, half_num);

    // Left part
    P_bd_L.setZero(3,half_num);
    P_bd_L = bd.block(0, I_start_y, 3, half_num);
    // Right part
    P_bd_R.setZero(3,half_num);
    P_bd_R.topLeftCorner(3, I_start_y) = bd.topLeftCorner(3, I_start_y);
    P_bd_R.bottomRightCorner(3, half_num-I_start_y) = bd.bottomRightCorner(3, half_num-I_start_y);

    P_bd.P_bd_L = P_bd_L;
    P_bd.P_bd_R = P_bd_R;

    return P_bd;
}

// Find the closest points with a sweep plane, and return separated z-coords
boundary3D::sepZ highwayRoadmap3D::closestPt(vector<MatrixXd> bd,
                                              double tx, vector<double> ty){
    boundary3D::sepZ z_bd;
    z_bd.z_L = MatrixXd::Constant(ty.size(), bd.size(),
                                  std::numeric_limits<double>::quiet_NaN());
    z_bd.z_R = z_bd.z_L;

    MatrixXd::Index I_L, I_R;
    VectorXd y(1);

    for(size_t i=0; i<bd.size(); i++){
        // Find points close to the given x-plane
        Matrix<double, 3, Dynamic> yz_bd;
        for(long k=0; k<bd[i].cols(); k++)
            if( fabs(bd[i](0,k) - tx) <= 0.1) yz_bd << bd[i].col(k);
        if(yz_bd.cols() == 0) continue;

        // Split the boundary of the plane into two parts
        boundary3D::sepBd P_bd = separateBoundary(yz_bd);
        if(P_bd.P_bd_L.cols() == 0 || P_bd.P_bd_R.cols() == 0) continue;

        // Sweep y-coords
        for(size_t j=0; j<ty.size(); j++){
            // For each ty, find closes point, ie the intersection btw sweep line and arena/obstacle
            y << ty[j];
            (P_bd.P_bd_L.row(1).colwise() - y).
                    colwise().squaredNorm().minCoeff(&I_L);
            (P_bd.P_bd_R.row(1).colwise() - y).
                    colwise().squaredNorm().minCoeff(&I_R);

            z_bd.z_L(j,i) = P_bd.P_bd_L(2,I_L);
            z_bd.z_R(j,i) = P_bd.P_bd_R(2,I_R);
        }
    }

    return z_bd;
}

cf_cellYZ highwayRoadmap3D::enhanceDecomp(cf_cellYZ cell){
    // Make sure all connections between vertexes are within one convex cell
    cf_cellYZ cell_new = cell;

    for(size_t i=0; i<cell.ty.size()-1; i++){
        for(size_t j1=0; j1<cell.xM[i].size(); j1++){
            for(size_t j2=0; j2<cell.xM[i+1].size(); j2++){
                if(cell_new.xM[i][j1] < cell_new.xL[i+1][j2] && cell_new.xU[i][j1] >= cell_new.xL[i+1][j2]){
                    cell_new.xU[i].push_back(cell_new.xL[i+1][j2]);
                    cell_new.xL[i].push_back(cell_new.xL[i+1][j2]);
                    cell_new.xM[i].push_back(cell_new.xL[i+1][j2]);
                }
                else if(cell_new.xM[i][j1] > cell_new.xU[i+1][j2] && cell_new.xL[i][j1] <= cell_new.xU[i+1][j2]){
                    cell_new.xU[i].push_back(cell_new.xU[i+1][j2]);
                    cell_new.xL[i].push_back(cell_new.xU[i+1][j2]);
                    cell_new.xM[i].push_back(cell_new.xU[i+1][j2]);
                }

                if(cell_new.xM[i+1][j2] < cell_new.xL[i][j1] && cell_new.xU[i+1][j2] >= cell_new.xL[i][j1]){
                    cell_new.xU[i+1].push_back(cell_new.xL[i][j1]);
                    cell_new.xL[i+1].push_back(cell_new.xL[i][j1]);
                    cell_new.xM[i+1].push_back(cell_new.xL[i][j1]);
                }
                else if(cell_new.xM[i+1][j2] > cell_new.xU[i][j1] && cell_new.xL[i+1][j2] <= cell_new.xU[i][j1]){
                    cell_new.xU[i+1].push_back(cell_new.xU[i][j1]);
                    cell_new.xL[i+1].push_back(cell_new.xU[i][j1]);
                    cell_new.xM[i+1].push_back(cell_new.xU[i][j1]);
                }
            }
        }

        sort(cell_new.xL[i].begin(), cell_new.xL[i].end(), [](double a, double b){return a < b;});
        sort(cell_new.xU[i].begin(), cell_new.xU[i].end(), [](double a, double b){return a < b;});
        sort(cell_new.xM[i].begin(), cell_new.xM[i].end(), [](double a, double b){return a < b;});
    }

    return cell_new;
}

vector<double> highwayRoadmap3D::addMidVtx(vector<double> vtx1, vector<double> vtx2){
    // Connect vertexes among different layers, and add a middle vertex to the roadmap
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

void highwayRoadmap3D::sampleSO3(){
    vector<double> u(3);

    // Uniform random samples for Quaternions
    for(size_t i=0; i<N_layers; i++){
        u = {rand()*0.5/RAND_MAX, rand()*0.5/RAND_MAX, rand()*0.5/RAND_MAX};
        this->q_r[i][0] = sqrt(1-u[0])*sin(2*pi*u[1]);
        this->q_r[i][1] = sqrt(1-u[0])*cos(2*pi*u[1]);
        this->q_r[i][2] = sqrt(u[0])*sin(2*pi*u[2]);
        this->q_r[i][3] = sqrt(u[0])*cos(2*pi*u[2]);
    }
}

unsigned int highwayRoadmap3D::find_cell(vector<double> v){
    // Find the cell that an arbitrary vertex locates, and find the closest roadmap vertex
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

double highwayRoadmap3D::vector_dist(vector<double> v1, vector<double> v2){
    vector<double> diff;
    for(size_t i=0; i<v1.size(); i++) diff.push_back(v1[i]-v2[i]);
    return sqrt( inner_product(diff.begin(), diff.end(), diff.begin(), 0.0) );
}

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

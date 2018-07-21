#include <iostream>
#include <list>
#include <random>
#include <src/planners/highwayroadmap.h>
#include <src/planners/interval.h>
#include <src/geometry/ptinpoly.h>

#include <fstream>

#define pi 3.1415926

highwayRoadmap::highwayRoadmap(vector<SuperEllipse> robot, polyCSpace vtxMat, vector< vector<double> > endpt, vector<SuperEllipse> arena, vector<SuperEllipse> obs, option opt){
    Robot = robot;
    Endpt = endpt;
    Arena = arena;
    Obs = obs;
    N_o = opt.N_o;
    N_s = opt.N_s;

    polyVtx = vtxMat;

    infla = opt.infla;
    N_layers = opt.N_layers;
    N_dy = opt.N_dy;
    N_KCsample = opt.sampleNum;

    Cost = 0;
}

void highwayRoadmap::plan(){
    time::point start = time::now();
    buildRoadmap();
    planTime.buildTime = time::seconds(time::now() - start);

    start = time::now();
    search();
    planTime.searchTime = time::seconds(time::now() - start);
}

void highwayRoadmap::buildRoadmap(){
    // angle steps
    double dr = pi/(N_layers-1);
    graph multiGraph;

    for(size_t i=0; i<N_layers; i++){
        Robot[0].a[2] = dr*i;
        // boundary for obstacles and arenas
        boundary bd = boundaryGen();

        // collision-free cells, stored by ty, xL, xU, xM
        cf_cell CFcell = rasterScan(bd.bd_s, bd.bd_o);

        // construct adjacency matrix for one layer
        connectOneLayer(CFcell);

        // Store the number of vertex before the current layer
        N_v_layer.push_back(vtxEdge.vertex.size());
    }
    connectMultiLayer();
}

boundary highwayRoadmap::boundaryGen(){
    SuperEllipse robot_infla = Robot[0];
    boundary bd;

    // Enlarge the robot
    robot_infla.a[0] *= 1+infla;
    robot_infla.a[1] *= 1+infla;

    // calculate Minkowski boundary points
    for(size_t i=0; i<N_s; i++)
        bd.bd_s.push_back( Arena[i].minkSum2D(Arena[i].a, robot_infla.a, Arena[i].num, -1) );
    for(size_t i=0; i<N_o; i++)
        bd.bd_o.push_back( Obs[i].minkSum2D(Obs[i].a, robot_infla.a, Obs[i].num, +1) );

    return bd;
}

cf_cell highwayRoadmap::rasterScan(vector<MatrixXd> bd_s, vector<MatrixXd> bd_o){
    cf_cell cell, cell_new;

    boundary::sepBd P_bd_s[N_s], P_bd_o[N_o];
    boundary::sepBd x_bd_s[N_dy][N_s], x_bd_o[N_dy][N_o];

    MatrixXd bd_s_L[N_s], bd_s_R[N_s], bd_o_L[N_o], bd_o_R[N_o];
    MatrixXd x_s_L(N_dy, N_s), x_s_R(N_dy, N_s);
    MatrixXd x_o_L(N_dy, N_o), x_o_R(N_dy, N_o);

    interval op;
    vector<Interval> cf_seg[N_dy], obs_seg, arena_seg, obs_merge, arena_inter;
    vector<double> xL, xU, xM;

    // Separate boundaries of Arenas and Obstacles into two parts
    for(size_t i=0; i<N_s; i++) {
        P_bd_s[i] = separateBoundary(bd_s[i]);
        bd_s_L[i] = P_bd_s[i].P_bd_L;
        bd_s_R[i] = P_bd_s[i].P_bd_R;
    }
    for(size_t i=0; i<N_o; i++) {
        P_bd_o[i] = separateBoundary(bd_o[i]);
        bd_o_L[i] = P_bd_o[i].P_bd_L;
        bd_o_R[i] = P_bd_o[i].P_bd_R;
    }

    // Find closest points for each raster scan line
    double ty[N_dy], dy = (P_bd_s[0].max_y-P_bd_s[0].min_y)/(N_dy-1);
    for(size_t i=0; i<N_dy; i++){
        // y-coordinate of each sweep line
        ty[i] = P_bd_s[0].min_y + i * dy;
        // x-coordinate of the intersection btw sweep line and arenas
        for(size_t j=0; j<N_s; j++) {
            x_bd_s[i][j] = closestPt(P_bd_s[j], ty[i]);
            x_s_L(i,j) = x_bd_s[i][j].x_L;
            x_s_R(i,j) = x_bd_s[i][j].x_R;
        }
        // x-coordinate of the intersection btw sweep line and obstacles
        for(size_t j=0; j<N_o; j++) {
            x_bd_o[i][j] = closestPt(P_bd_o[j], ty[i]);
            x_o_L(i,j) = x_bd_o[i][j].x_L;
            x_o_R(i,j) = x_bd_o[i][j].x_R;
        }
    }

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

    // CF line segment for each ty
    for(size_t i=0; i<N_dy; i++){
        // Construct intervals at each sweep line
        for(size_t j=0; j<N_s; j++) if(!isnan(x_s_L(i,j)) && !isnan(x_s_R(i,j)))
            arena_seg.push_back({x_s_L(i,j), x_s_R(i,j)});
        for(size_t j=0; j<N_o; j++) if(!isnan(x_o_L(i,j)) && !isnan(x_o_R(i,j)))
            obs_seg.push_back({x_o_L(i,j), x_o_R(i,j)});

        // y-coord
        cell.ty.push_back(ty[i]);

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
        cell.xL.push_back(xL);
        cell.xU.push_back(xU);
        cell.xM.push_back(xM);

        // Clear memory
        arena_seg.clear();
        obs_seg.clear();
        xL.clear();
        xU.clear();
        xM.clear();
    }

    // Enhanced cell decomposition
    cell_new = enhanceDecomp(cell);

    return cell_new;
}

void highwayRoadmap::connectOneLayer(cf_cell CFcell){
    // Construct a vector of vertex
    for(size_t i=0; i<CFcell.ty.size(); i++){
        size_t N_0 = vtxEdge.vertex.size();
        for(size_t j=0; j<CFcell.xM[i].size(); j++){
            vtxEdge.vertex.push_back({CFcell.xM[i][j], CFcell.ty[i], Robot[0].a[2]});

            // Connect vertex within one sweep line
//            if(j != CFcell.xM[i].size()-1){
//                if(abs(CFcell.xU[i][j] - CFcell.xL[i][j+1]) < 1e-5){
//                    vtxEdge.edge.push_back(make_pair(N_0+j, N_0+j+1));
//                    vtxEdge.weight.push_back(1.0);
//                }
//            }
        }
        size_t N_1 = vtxEdge.vertex.size();

        if(i != CFcell.ty.size()-1){
        // Connect vertex btw adjacent cells
            for(size_t j1=0; j1<CFcell.xM[i].size(); j1++){
                for(size_t j2=0; j2<CFcell.xM[i+1].size(); j2++){
                    if( ( (CFcell.xM[i][j1] > CFcell.xL[i+1][j2] && CFcell.xM[i][j1] < CFcell.xU[i+1][j2]) ||
                        (CFcell.xM[i+1][j2] > CFcell.xL[i][j1] && CFcell.xM[i+1][j2] < CFcell.xU[i][j1]) ) ||
                        ( (CFcell.xU[i][j1] > CFcell.xL[i+1][j2] && CFcell.xU[i][j1] < CFcell.xU[i+1][j2]) ||
                        (CFcell.xL[i][j1] > CFcell.xL[i+1][j2] && CFcell.xL[i][j1] < CFcell.xU[i+1][j2]) ||
                        (CFcell.xU[i+1][j2] > CFcell.xL[i][j1] && CFcell.xU[i+1][j2] < CFcell.xU[i][j1]) ||
                        (CFcell.xL[i+1][j2] > CFcell.xL[i][j1] && CFcell.xL[i+1][j2] < CFcell.xU[i][j1]) ) ){
                            vtxEdge.edge.push_back(make_pair(N_0+j1, N_1+j2));
                            vtxEdge.weight.push_back(1.0);
                    }
                }
            }
        }
    }
}

void highwayRoadmap::connectMultiLayer(){
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
                d = pow((v1[0]-v2[0]),2.0)+pow((v1[1]-v2[1]),2.0);
                if(d<=0.1) midVtx = addMidVtx(v1,v2);
                if(!midVtx.empty()){
                    vtxEdge.vertex.push_back(midVtx);

                    vtxEdge.edge.push_back(make_pair(m, n));
                    vtxEdge.weight.push_back(1.0);
                    vtxEdge.edge.push_back(make_pair(m2, n));
                    vtxEdge.weight.push_back(1.0);
                    n++;
                    break;
                }
            }
            midVtx.clear();
        }
        start = n_11;
    }
}

void highwayRoadmap::search(){
    int idx_s, idx_g, num;

    // Construct the roadmap
    int num_vtx = vtxEdge.vertex.size();
    AdjGraph g(num_vtx);

    for(size_t i=0; i<vtxEdge.edge.size(); i++)
        add_edge(vtxEdge.edge[i].first, vtxEdge.edge[i].second, Weight(vtxEdge.weight[i]) ,g);

//    property_map<AdjGraph, edge_weight_t>::type weightmap = get(edge_weight, g);

    // Locate the nearest vertex for start and goal in the roadmap
    idx_s = find_cell(Endpt[0]);
    idx_g = find_cell(Endpt[1]);

    // Search for shortest path
    std::vector<vertex_descriptor> p(num_vertices(g));
    std::vector<double> d(num_vertices(g));

    dijkstra_shortest_paths(g, idx_g,
                            predecessor_map(make_iterator_property_map(p.begin(), get(vertex_index, g))).
                            distance_map(make_iterator_property_map(d.begin(), get(vertex_index, g))));

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
boundary::sepBd highwayRoadmap::separateBoundary(MatrixXd bd){
    const int half_num = Arena[0].num/2;
    MatrixXd::Index I_max_y, I_min_y;
    int I_start_y;
    MatrixXd P_bd_L, P_bd_R;
    double max_y, min_y;
    boundary::sepBd P_bd;

    // Find separating point
    max_y = bd.row(1).maxCoeff(&I_max_y);
    min_y = bd.row(1).minCoeff(&I_min_y);
    I_start_y = min(I_max_y, I_min_y);
    I_start_y = min(I_start_y, half_num);

    // Left part
    P_bd_L.setZero(2,half_num);
    P_bd_L = bd.block(0, I_start_y, 2, half_num);
    // Right part
    P_bd_R.setZero(2,half_num);
    P_bd_R.topLeftCorner(2, I_start_y) = bd.topLeftCorner(2, I_start_y);
    P_bd_R.bottomRightCorner(2, half_num-I_start_y) = bd.bottomRightCorner(2, half_num-I_start_y);

    P_bd.P_bd_L = P_bd_L;
    P_bd.P_bd_R = P_bd_R;
    P_bd.max_y = max_y;
    P_bd.min_y = min_y;

    return P_bd;
}

boundary::sepBd highwayRoadmap::closestPt(boundary::sepBd P_bd, double ty){
    boundary::sepBd x_bd;
    MatrixXd::Index I_L, I_R;
    VectorXd y(1);

    // check if ty in the range of each arena/obstacle
    if( (ty > P_bd.max_y) || (ty < P_bd.min_y) ){
        x_bd.x_L = numeric_limits<double>::quiet_NaN();
        x_bd.x_R = numeric_limits<double>::quiet_NaN();
        return x_bd;
    }
    // For each ty, find closes point, ie the intersection btw sweep line and arena/obstacle
    y << ty;
    (P_bd.P_bd_L.row(1).colwise() - y).colwise().squaredNorm().minCoeff(&I_L);
    (P_bd.P_bd_R.row(1).colwise() - y).colwise().squaredNorm().minCoeff(&I_R);

    x_bd.x_L = P_bd.P_bd_L(0,I_L);
    x_bd.x_R = P_bd.P_bd_R(0,I_R);
    return x_bd;
}

MatrixXd highwayRoadmap::boundaryEnlarge(MatrixXd bd_o[], MatrixXd x_o, double ty[], int K){
    // Enclose the curved boundaries of c-obstacles by polyhedrons
    MatrixXd x_o_Ex(N_dy, N_o);
    double x_Ex;
    double d;

    // Initialize x_o_Ex as NaN matrix
    for(size_t j=0; j<N_o; j++){
        for(size_t i=0; i<N_dy; i++){
            x_o_Ex(i,j) = numeric_limits<double>::quiet_NaN();
        }
    }

    // Compute tangent line and enlarge the boundary
    for(size_t j=0; j<N_o; j++){
        int count = 0;

        for(size_t i=0; i<N_dy-1; i++){
            double dist = 0, phi;

            if( isnan(x_o(i,j)) || isnan(x_o(i+1,j)) ) continue;
            count += 1;
            double p1[2] = {x_o(i,j), ty[i]};
            double p2[2] = {x_o(i+1,j), ty[i+1]};

            // Search for farthest point to the line segment def by two intersecting points
            for(size_t k=0; k<sizeof(bd_o[j]); k++){
                double p[2] = {bd_o[j](0,k), bd_o[j](1,k)};

                if( (p[1] > ty[i]) && (p[1] < ty[i+1]) ){
                    d = abs( (p2[1]-p1[1])*p[0] - (p2[0]-p1[0])*p[1] + p2[0]*p1[1] - p2[1]*p1[0] )
                            / sqrt( pow((p2[1]-p1[1]),2) + pow((p2[0]-p1[0]),2) );
                    if(d > dist) dist = d;
                }
            }
            phi = atan2( p2[1]-p1[1], p2[0]-p1[0] );
            x_Ex = x_o(i,j) + K * dist/sin(phi);

            // Update the boundary point to be farthest to the previous one
            if(K == -1) {if(x_Ex <= x_o(i,j)) x_o_Ex(i,j) = x_Ex;}
            else if(K == +1) {if(x_Ex >= x_o(i,j)) x_o_Ex(i,j) = x_Ex;}

            if(count == 1) x_o_Ex(i,j) = x_o(i,j) + K * dist/sin(phi);
            x_o_Ex(i+1,j) = x_o(i+1,j) + K * dist/sin(phi);
        }
    }

    return x_o_Ex;
}

cf_cell highwayRoadmap::enhanceDecomp(cf_cell cell){
    // Make sure all connections between vertexes are within one convex cell
    cf_cell cell_new = cell;

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

vector<double> highwayRoadmap::addMidVtx(vector<double> vtx1, vector<double> vtx2){
    // Connect vertexes among different layers, and add a middle vertex to the roadmap
    vector<double> midVtx, pt, pt1, pt2;
    ptInPoly polyTest;
    bool flag;
    midVtx.clear();

    for(size_t iter = 0; iter<N_KCsample; iter++){
        pt.clear(); pt1.clear(); pt2.clear();
        for(size_t i=0; i<vtx1.size(); i++){
            pt.push_back( (rand() * (vtx1[i]-vtx2[i]))/RAND_MAX + vtx2[i] );
            pt1.push_back(pt[i] - vtx1[i]);
            pt2.push_back(pt[i] - vtx2[i]);
        }
        flag = polyTest.isPtInPoly(polyVtx, pt1);
        if(flag){
            flag = polyTest.isPtInPoly(polyVtx, pt2);
            if(flag){
                midVtx = pt;
                return midVtx;
            }
        }
    }

    return midVtx;
}

double highwayRoadmap::vector_dist(vector<double> v1, vector<double> v2){
    vector<double> diff;
    for(size_t i=0; i<v1.size(); i++) diff.push_back(v1[i]-v2[i]);
    return sqrt( inner_product(diff.begin(), diff.end(), diff.begin(), 0.0) );
}

int highwayRoadmap::find_cell(vector<double> v){
    // Find the cell that an arbitrary vertex locates, and find the closest roadmap vertex
    double d_min, d;
    int idx = 0;

    d_min = vector_dist(v,vtxEdge.vertex[0]);
    for(int i=0; i<vtxEdge.vertex.size(); i++){
        d = vector_dist(v,vtxEdge.vertex[i]);
        if(d < d_min){
            d_min = d;
            idx = i;
        }
    }

    return idx;
}

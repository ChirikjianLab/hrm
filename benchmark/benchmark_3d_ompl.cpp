#include "ompl/ompl_planner.h"

EMesh getMesh(SuperQuadrics sq){
    sq.Shape.q.setIdentity();
    sq.Shape.pos[0] = 0.0; sq.Shape.pos[1] = 0.0; sq.Shape.pos[2] = 0.0;

    EMesh M;
    MeshGenerator MeshGen;
    ParametricPoints pts = MeshGen.getBoundary3D(sq);
    M = MeshGen.getMesh(pts);
    return M;
}

vector<SuperQuadrics> generateSQ(string file_name, double D){
    // Read config file
    inputFile file;
    vector< vector<double> > config = file.parse2DCsvFile(file_name);

    // Generate SQ object
    vector<SuperQuadrics> obj(config.size());
    for(size_t j = 0; j < config.size(); j++){
        obj[j].Shape.a[0] = config[j][0];
        obj[j].Shape.a[1] = config[j][1];
        obj[j].Shape.a[2] = config[j][2];
        obj[j].Shape.eps[0] = config[j][3];
        obj[j].Shape.eps[1] = config[j][4];
        obj[j].Shape.pos[0] = config[j][5];
        obj[j].Shape.pos[1] = config[j][6];
        obj[j].Shape.pos[2] = config[j][7];
        obj[j].Shape.q.w() = config[j][8];
        obj[j].Shape.q.x() = config[j][9];
        obj[j].Shape.q.y() = config[j][10];
        obj[j].Shape.q.z() = config[j][11];
        obj[j].n = long(D);
    }
    return obj;
}

int main(int argc, char ** argv){
    if (argc != 5) {
        cerr << "Usage: Please add 1) Num of trials 2) Param for vertex 3) Planner 4) Sampler" << endl;
        return 1;
    }

    // Record planning time for N trials
    int N = atoi(argv[1]);
    double n = atof(argv[2]);

    vector< vector<double> > time_stat;

    // Read and setup environment config
    string robot_config = "../config/robot_config_3d.csv",
            arena_config = "../config/arena_config_3d.csv",
            obs_config = "../config/obs_config_3d.csv";
    vector<SuperQuadrics> robot = generateSQ(robot_config, n),
            arena = generateSQ(arena_config, n),
            obs = generateSQ(obs_config, n);

    // Obstacle mesh
    vector<EMesh> obs_mesh;
    for(size_t i=0; i<obs.size(); i++) obs_mesh.push_back( getMesh(obs[i]) );

    // Boundary
    double f = 1.5;
    vector<double> b1 = {-arena[0].Shape.a[0] + f * robot[0].Shape.a[0],
                         -arena[0].Shape.a[1] + f * robot[0].Shape.a[0],
                         -arena[0].Shape.a[2] + f * robot[0].Shape.a[0]},
            b2 = {arena[0].Shape.a[0] - f * robot[0].Shape.a[0],
                  arena[0].Shape.a[1] - f * robot[0].Shape.a[0],
                  arena[0].Shape.a[2] - f * robot[0].Shape.a[0]};

    // Start and goal setup
    inputFile file;
    string file_endpt = "../config/endPts_3d.csv";
    vector<vector<double> > endPts = file.parse2DCsvFile(file_endpt);

    std::ofstream outfile;
    outfile.open("time3D_ompl.csv");
    outfile << "PLANNER" << ',' << "SAMPLER" << ',' << "SUCCESS" << ',' << "TOTAL_TIME" << ',' <<
               "GRAPH_NODES" << ',' << "GRAPH_EDGES" << ',' << "PATH_CONFIG" << ',' << "VALID_SPACE" << endl;

    // Planner number: PRM:0, LazyPRM:1, RRT:2, RRTconnect:3, EST:4, KPIECE:5
    // Sampler number: Uniform:0, OB:1, Gaussian:2, MaxClearance:3, Bridge:4
    int id_plan = atoi(argv[3]), id_sample = atoi(argv[4]);
//    int m = id_plan, nn = id_sample;
    for (int m=0; m<id_plan; m++) {
        for (int n=0; n<id_sample; n++) {
            for(int i=0; i<N; i++){
                cout << "Planner: " << m << endl;
                cout << "Sampler: " << n << endl;
                cout << "Num of trials: " << i << endl;

                ompl_planner tester(b1, b2, robot, arena, obs, obs_mesh, m, n);
                tester.plan(endPts[0], endPts[1]);

                outfile << tester.id_planner << ',' << tester.id_sampler << ',' << tester.flag << ',' << tester.total_time << ',' <<
                           tester.nodes_graph <<","<< tester.edges_graph <<","<< tester.nodes_path <<","<< tester.valid_space << endl;
            }
        }
    }
    outfile.close();

    return 0;
}

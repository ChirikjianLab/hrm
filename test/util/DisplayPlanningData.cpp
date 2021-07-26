#include "include/DisplayPlanningData.h"

#include <fstream>
#include <iostream>

void displayPlanningTimeInfo(const Time* time) {
    std::cout << "Roadmap build time: " << time->buildTime << "s" << std::endl;
    std::cout << "Path search time: " << time->searchTime << "s" << std::endl;
    std::cout << "Total Planning Time: " << time->totalTime << 's' << std::endl;
}

void displayGraphInfo(const Graph* graph, const bool isStore) {
    std::cout << "Number of valid configurations: " << graph->vertex.size()
              << std::endl;
    std::cout << "Number of valid edges: " << graph->edge.size() << std::endl;

    // Write the output to .csv files
    if (isStore) {
        std::ofstream fileVtx;
        fileVtx.open("vertex_3D.csv");
        std::vector<std::vector<double>> vtx = graph->vertex;
        for (size_t i = 0; i < vtx.size(); ++i) {
            for (size_t j = 0; i < vtx.at(i).size(); ++j) {
                fileVtx << vtx[i][j] << ',';
            }
            fileVtx << "\n";
        }
        fileVtx.close();

        std::ofstream fileEdge;
        fileEdge.open("edge_3D.csv");
        for (size_t i = 0; i < graph->edge.size(); ++i) {
            fileEdge << graph->edge.at(i).first << ','
                     << graph->edge.at(i).second << "\n";
        }
        fileEdge.close();
    }
}

void displayPathInfo(const SolutionPathInfo* path, const bool isStore) {
    std::cout << "Path length: " << path->PathId.size() << std::endl;
    std::cout << "Path cost: " << path->cost << std::endl;

    // Write the output to .csv files
    if (isStore) {
        std::ofstream filePathId;
        filePathId.open("paths_3D.csv");
        if (!path->PathId.empty()) {
            for (size_t i = 0; i < path->PathId.size(); i++) {
                filePathId << path->PathId[i] << ',';
            }
        }
        filePathId.close();

        // Retrieve solution path
        std::ofstream filePath;
        filePath.open("solution_path_3D.csv");
        for (size_t i = 0; i < path->solvedPath.size(); ++i) {
            for (size_t j = 0; j < path->solvedPath.at(i).size(); ++j) {
                filePath << path->solvedPath.at(i).at(j) << ',';
            }
            filePath << "\n";
        }
        filePath.close();

        std::ofstream fileInterpPath;
        fileInterpPath.open("interpolated_path_3D.csv");
        for (size_t i = 0; i < path->interpolatedPath.size(); ++i) {
            for (size_t j = 0; j < path->interpolatedPath.at(i).size(); ++j) {
                fileInterpPath << path->interpolatedPath.at(i).at(j) << ',';
            }
            fileInterpPath << "\n";
        }
        fileInterpPath.close();
    }
}

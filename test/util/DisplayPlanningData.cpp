#include "include/DisplayPlanningData.h"

#include <fstream>
#include <iostream>

void displayPlanningTimeInfo(const Time* time) {
    std::cout << "Roadmap build time: " << time->buildTime << "s" << std::endl;
    std::cout << "Path search time: " << time->searchTime << "s" << std::endl;
    std::cout << "Total Planning Time: " << time->totalTime << 's' << std::endl;
}

void displayGraphInfo(const Graph* graph) {
    std::cout << "Number of valid configurations: " << graph->vertex.size()
              << std::endl;
    std::cout << "Number of valid edges: " << graph->edge.size() << std::endl;
}

void displayGraphInfo(const Graph* graph, const std::string& dim) {
    displayGraphInfo(graph);

    // Write the output to .csv files

    std::ofstream fileVtx;
    fileVtx.open("vertex_" + dim + ".csv");
    std::vector<std::vector<double>> vertexList = graph->vertex;
    for (const auto& vertex : vertexList) {
        for (const auto& vtx : vertex) {
            fileVtx << vtx << ',';
        }
        fileVtx << "\n";
    }
    fileVtx.close();

    std::ofstream fileEdge;
    fileEdge.open("edge_" + dim + ".csv");
    for (auto edge : graph->edge) {
        fileEdge << edge.first << ',' << edge.second << "\n";
    }
    fileEdge.close();
}

void displayPathInfo(const SolutionPathInfo* path) {
    std::cout << "Path length: " << path->PathId.size() << std::endl;
    std::cout << "Path cost: " << path->cost << std::endl;
}

void displayPathInfo(const SolutionPathInfo* path, const std::string& dim) {
    displayPathInfo(path);

    // Write the output to .csv files
    std::ofstream filePathId;
    filePathId.open("path_id_" + dim + ".csv");
    if (!path->PathId.empty()) {
        for (auto pathId : path->PathId) {
            filePathId << pathId << ',';
        }
    }
    filePathId.close();

    // Retrieve solution path
    std::ofstream filePath;
    filePath.open("solution_path_" + dim + ".csv");
    for (const auto& solvedPath : path->solvedPath) {
        for (const auto& solvedPathVal : solvedPath) {
            filePath << solvedPathVal << ',';
        }
        filePath << "\n";
    }
    filePath.close();

    std::ofstream fileInterpPath;
    fileInterpPath.open("interpolated_path_" + dim + ".csv");
    for (const auto& interpPath : path->interpolatedPath) {
        for (const auto& interpPathVal : interpPath) {
            fileInterpPath << interpPathVal << ',';
        }
        fileInterpPath << "\n";
    }
    fileInterpPath.close();
}
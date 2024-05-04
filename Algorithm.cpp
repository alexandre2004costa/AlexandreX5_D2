//
// Created by sofia on 02/05/2024.
//

#include <limits>
#include <vector>
#include "Algorithm.h"

using namespace std;


bool isPathComplete(Graph<int>& graph, vector<unsigned int>& currentPath) {
    int n = currentPath.size() - 1;
    for (auto edge: graph.findVertex(currentPath[n])->getAdj())
        if (edge->getDest()->getInfo() == 0)
            return true;
    return false;
}


unsigned int calculateDistance(Graph<int>& graph, const vector<unsigned int>& path) {
    int n = path.size() - 1;
    unsigned int distance = 0;
    for (int i = 0; i < n; ++i) {
        for (auto edge: graph.findVertex(path[i])->getAdj())
            if (edge->getDest()->getInfo() == path[i+1])
                distance += edge->getWeight();
    }

    for (auto edge: graph.findVertex(path[n])->getAdj())
        if (edge->getDest()->getInfo() == 0)
            distance += edge->getWeight();

    return distance;
}


void backtrack(Graph<int>& graph, vector<unsigned int>& currentPath, vector<unsigned int>& bestPath, unsigned int& minDistance) {
    int n = graph.getNumVertex();

    if (currentPath.size() == n) {   // A complete path
        if (isPathComplete(graph, currentPath)) {
            unsigned int distance = calculateDistance(graph, currentPath);
            if (distance < minDistance) {
                bestPath = currentPath;
                minDistance = distance;
            }
        }

    } else {
        for (unsigned int i = 1; i < n; i++) {      // Start from 1 since *
            if (!graph.findVertex(i)->isVisited()) {
                graph.findVertex(i)->setVisited(true);
                currentPath.push_back(i);

                backtrack(graph, currentPath, bestPath, minDistance);

                graph.findVertex(i)->setVisited(false);
                currentPath.pop_back();
            }
        }
    }
}


unsigned int Algorithm::Backtracking(Graph<int>& graph, vector<unsigned int>& minPath) {
    for (auto vertex: graph.getVertexSet()) {
        vertex->setVisited(false);
        vertex->setDist(0);
    }

    graph.findVertex(0)->setVisited(true);
    vector<unsigned int> currentPath = {0};     // Start from vertex 0 *
    vector<unsigned int> bestPath;
    unsigned int minDistance = numeric_limits<unsigned int>::max();

    backtrack(graph, currentPath, bestPath, minDistance);

    minPath = bestPath;
    return minDistance;
}
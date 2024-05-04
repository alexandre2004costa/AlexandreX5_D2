
#include "Menu.h"

bool existsPath(Graph<int>& graph, int infoOrig, int infoDest) {
    for (auto edge: graph.findVertex(infoOrig)->getAdj())
        if (edge->getDest()->getInfo() == infoDest)
            return true;
    return false;
}

double calculateDistance(Graph<int>& graph, int infoLast, int infoNext) {
    double distance = graph.findVertex(infoLast)->getDist();

    for (auto edge: graph.findVertex(infoLast)->getAdj())
        if (edge->getDest()->getInfo() == infoNext)
            return (distance + edge->getWeight());
}


void backtrack(Graph<int>& graph, vector<int>& currentPath, vector<int>& bestPath, double& minDistance) {
    int pathSize = currentPath.size(), numVertex = graph.getNumVertex();

    if (pathSize == numVertex) {   // A complete path
        if (existsPath(graph, currentPath[pathSize-1], 0)) {   // Verify if exists path to zero
            double distance = calculateDistance(graph, currentPath[pathSize-1], 0);
            //cout << "Dist: " << distance << endl;
            if (distance < minDistance) {
                bestPath = currentPath;
                minDistance = distance;
            }
        }

    } else {
        for (int i = 1; i < numVertex; i++) {      // Start from 1 since *
            Vertex<int>* nextVert = graph.findVertex(i);

            if (existsPath(graph, currentPath[pathSize - 1], i) && !nextVert->isVisited()) {
                nextVert->setVisited(true);
                double dist = calculateDistance(graph, currentPath[pathSize - 1], i);
                nextVert->setDist(dist);
                currentPath.push_back(i);

                backtrack(graph, currentPath, bestPath, minDistance);

                nextVert->setVisited(false);
                currentPath.pop_back();
            }
        }
    }
}


double Menu::Backtracking(Graph<int>& graph, vector<int>& minPath) {
    for (auto vertex: graph.getVertexSet()) {
        vertex->setVisited(false);
        vertex->setDist(0);
    }

    graph.findVertex(0)->setVisited(true);
    vector<int> currentPath = {0};     // Start from vertex 0 *
    double minDistance = numeric_limits<double>::max();

    backtrack(graph, currentPath, minPath, minDistance);
    return minDistance;
}


#include <vector>
#include "DataStructures/Graph.h"
#include "Menu.h"
#include <random>
#include <stack>
#include <set>


template <class T>
bool isCycle(Vertex<T> *s, Vertex<T> *t, int size){
    if (s == t) return true;
    size--;
    queue<Vertex<int> *> q;
    q.push(s);
    while (size > 1 && !q.empty()){
        s = q.front();
        q.pop();
        for (Edge<T> *e : s->getConnects()){
            auto v = e->getVertex(s);
            if (v->isVisited()) continue;
            if (v == t) return true;
            q.push(v);
        }
        s->setVisited(true);
        size--;
    }
    return false;
}

double Menu::greedyHeuristica(Graph<int> * g, vector<int>& minPath){
    std::vector<Edge<int> *> allEdges;
    for (auto v: g->getVertexSet()) {
        v->cleanConnect();
        for (auto e: v->getAdj()) {
            allEdges.push_back(e);
        }
    }
    sort(allEdges.begin(), allEdges.end(), [](Edge<int> *a, Edge<int> *b) { return a->getWeight() < b->getWeight(); });
    double totalWeight = 0;
        for (auto e : allEdges) {
            auto v1 = e->getPair().first;
            auto v2 = e->getPair().second;
            for (auto v: g->getVertexSet()) v->setVisited(false);
            if (v1->getConnects().size() >= 2 || v2->getConnects().size() >= 2) continue;
            if (v1->getConnects().size() == 0 || v2->getConnects().size() == 0
                || !isCycle(v2,v1, g->getVertexSet().size())) {
                    totalWeight += e->getWeight();
                    v1->addConnect(e);
                    v2->addConnect(e);
                }
        }
    auto v = g->findVertex(0);
    int lastV = 0;
    do{
        minPath.push_back(v->getInfo());
        for (auto k : v->getConnects()){
            if (k->getVertex(v)->getInfo() != lastV){
                lastV = v->getInfo();
                v = k->getVertex(v);
                break;
            }
        }
    } while (v->getInfo() != 0);
    return totalWeight;
}


double Menu::randomSwap(Graph<int> * g, vector<int>& minPath, double minDist) {
    double minDistS = minDist;
    int size = g->getNumVertex()-1;

    random_device rd; mt19937 gen(rd());
    uniform_int_distribution<int> dist(0, size);
    int pos1 = dist(gen), pos2 = dist(gen);
    while (pos1 == pos2) pos2 = dist(gen);

    int n1 = minPath[pos1], n2 = minPath[pos2];
    int bef1, aft1, bef2, aft2;

    if (pos1 == 0) bef1 = minPath[size - 1];
    else bef1 = minPath[pos1 - 1];
    if (pos2 == 0) bef2 = minPath[size - 1];
    else bef2 = minPath[pos2 - 1];
    if (pos1 == size) aft1 = minPath[1];
    else aft1 = minPath[pos1 + 1];
    if (pos2 == size) aft2 = minPath[1];
    else aft2 = minPath[pos2 + 1];

    for (auto e: g->findVertex(n1)->getAdj()) {
        auto v = e->getPair().second;
        double w = e->getWeight();

        if (v->getInfo() == bef1 || v->getInfo() == aft1)  minDistS -= w;
        else if (v->getInfo() == bef2 || v->getInfo() == aft2)  minDistS += w;
    }

    for (auto e: g->findVertex(n2)->getAdj()) {
        auto v = e->getPair().second;
        double w = e->getWeight();

        if (v->getInfo() == bef2 || v->getInfo() == aft2)  minDistS -= w;
        else if (v->getInfo() == bef1 || v->getInfo() == aft1)  minDistS += w;
    }


    if (minDistS < minDist) {
        minPath[pos1] = n2; minPath[pos2] = n1;
        return minDistS;
    }
    return minDist;
}




bool existsPath(Graph<int>& graph, int infoOrig, int infoDest) {
    for (auto edge: graph.findVertex(infoOrig)->getAdj())
        if (edge->getVertex(graph.findVertex(infoOrig))->getInfo() == infoDest)
            return true;
    return false;
}

double calculateDistance(Graph<int>& graph, int infoLast, int infoNext) {
    double distance = graph.findVertex(infoLast)->getDist();

    for (auto edge: graph.findVertex(infoLast)->getAdj())
        if (edge->getVertex(graph.findVertex(infoLast))->getInfo() == infoNext)
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


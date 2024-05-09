
#include <vector>
#include "DataStructures/Graph.h"
#include "Menu.h"
#include <stack>
#include <set>
#include <cmath>
#include <unordered_map>


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

double pi=3.14159265358979323846;
double earthradius=6371000; //meters

double Menu::toRadians(double coord){
    return coord*pi/180.0;
}

double Menu::haversineDistance(double lat1, double lon1, double lat2, double lon2){
    double radLat1=toRadians(lat1);
    double radLon1=toRadians(lon1);
    double radLat2=toRadians(lat2);
    double radLon2=toRadians(lon2);

    double deltaLat=radLat2-radLat1;
    double deltaLon=radLon2-radLon1;

    double a=sin(deltaLat/2)*sin(deltaLat/2)+cos(radLat1)*cos(radLat2)*sin(deltaLon/2)*sin(deltaLon/2);
    double c=2.0*atan2(sqrt(a), sqrt(1.0-a));

    return earthradius*c;
}


//PRIMMMMMMMMMMMMMMMMMMM

vector<Vertex<int>*> Menu::prim(Graph<int> * g){
    if (g->getVertexSet().empty()) {
        return g->getVertexSet();
    }

    for(auto v : g->getVertexSet()) {
        v->setDist(INF);
        v->setPath(nullptr);
        v->setVisited(false);
    }

    Vertex<int>* s = g->getVertexSet().front();
    s->setDist(0);

    MutablePriorityQueue<Vertex<int>> q;
    q.insert(s);

    while(!q.empty()) {

        auto v = q.extractMin();
        v->setVisited(true);

        for(auto &e : v->getAdj()) {
            Vertex<int>* w = e->getVertex(v);

            if (!w->isVisited()) {
                auto oldDist = w->getDist();

                if(e->getWeight() < oldDist) {
                    w->setDist(e->getWeight());
                    w->setPath(e);

                    if (oldDist == INF) {
                        q.insert(w);
                    }

                    else {
                        q.decreaseKey(w);
                    }
                }
            }
        }
    }
    return g->getVertexSet();
}

double Menu::triangularApproximationHeuristic(Graph<int>* g){
    return 0;
}



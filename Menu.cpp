
#include <vector>
#include "DataStructures/Graph.h"
#include "Menu.h"
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
            if (v->getInfo() < e->getVertex(v)->getInfo()) allEdges.push_back(e);
        }
    }
    sort(allEdges.begin(), allEdges.end(), [](Edge<int> *a, Edge<int> *b) { return a->getWeight() < b->getWeight(); });
    double totalWeight = 0;
        for (auto e : allEdges) {
            auto v1 = e->getPair().first;
            auto v2 = e->getPair().second;
            cout << v1->getInfo() << " : " << v2->getInfo() << endl;
            if (v1->getConnects().size() >= 2 || v2->getConnects().size() >= 2) continue;
            for (auto v: g->getVertexSet()) v->setVisited(false);
            if (v1->getConnects().size() == 0 || v2->getConnects().size() == 0
                || !isCycle(v2,v1, g->getVertexSet().size())) {
                    cout << "entrou" << endl;
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
template <class T>
void prim(Graph<T> * g) {
    for(auto v : g->getVertexSet()) {
        v->setDist(INF);
        v->setPath(nullptr);
        v->setVisited(false);
    }
    Vertex<T>* s = g->getVertexSet().front();
    s->setDist(0);
    MutablePriorityQueue<Vertex<T>> q;
    q.insert(s);
    while( ! q.empty() ) {
        auto v = q.extractMin();
        v->setVisited(true);
        for(auto &e : v->getAdj()) {
            Vertex<T>* w = e->getVertex(v);
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
}

template <class T>
void BestMatch(Graph<T> *g) {
    Vertex<int> * closest;
    int length;
    vector<Vertex<int> *>::iterator tmp, first;
    vector<Vertex<int> *> oddVertex;
    for (auto v : g->getVertexSet()) v->setDist(0);
    for (auto v : g->getVertexSet()){
        if (v->getPath() == nullptr) continue;
        v->setDist(v->getDist() + 1);
        v->getPath()->getVertex(v)->setDist(v->getPath()->getVertex(v)->getDist() + 1);
    }
    for (auto v : g->getVertexSet()){
        if (static_cast<int>(v->getDist()) % 2 != 0) oddVertex.push_back(v);
        else{
            if (v->getPath() == nullptr) continue;
            for (auto e : v->getAdj()){
                if (e == v->getPath()){
                    v->addConnect(e);
                    e->getVertex(v)->addConnect(e);
                }
            }
        }
        v->setDist(0);
    }

    /*while (!oddVertex.empty()) {
        first = oddVertex.begin();
        vector<Vertex<int> *>::iterator it = oddVertex.begin() + 1;
        vector<Vertex<int> *>::iterator end = oddVertex.end();
        length = std::numeric_limits<int>::max();
        for (; it != end; ++it) {
            for (auto e : (*it)->getAdj()) if (e->getVertex(*it) == *first){
                if (e->getWeight() < length) {
                    length =e->getWeight();
                    closest = *it;
                    tmp = it;
                }
            }

        }
        Edge<T> *e = new Edge(*first, closest, length);
        (*first)->addConnect(e);
        closest->addConnect(e);
        oddVertex.erase(tmp);
        oddVertex.erase(first);
    }*/

    double p = 0;
    for (auto v : g->getVertexSet()){
        cout << v->getInfo() << endl;
        for (auto e : v->getConnects()){
            if (e->getVertex(v)->getInfo() < v->getInfo()) {
                cout <<" ! "<< e->getVertex(v)->getInfo() << endl;
                p += e->getWeight();
            }
        }
    }
    cout << p;
}

template <class T>
void findEulerianCircuit(Vertex<T>* start, vector<Edge<T>*>& circuit) {
    stack<Vertex<T>*> stack;
    stack.push(start);

    while (!stack.empty()) {
        Vertex<T>* currentVertex = stack.top();
        bool found = false;

        for (auto& edge : currentVertex->getConnects()) {
            Vertex<T>* nextVertex = edge->getVertex(currentVertex);

            if (!nextVertex->isVisited()) {
                nextVertex->setVisited(true);
                stack.push(nextVertex);
                found = true;
                break;
            }
        }

        if (!found) {
            stack.pop();
            circuit.push_back(currentVertex->getPath());
        }
    }
}


double Menu::Cristofides(Graph<int> * g, vector<int>& minPath){
    prim(g);
    BestMatch(g);

}


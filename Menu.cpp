
#include <vector>
#include "DataStructures/Graph.h"
#include "Menu.h"
#include <stack>
#include <set>

/*
template <class T>
void prim(Graph<int> * g) {
    if (g->getVertexSet().empty()) {
        return;
    }
    for(auto v : g->getVertexSet()) {
        v->setDist(INF);
        v->setPath(nullptr);
        v->setVisited(false);
    }

    Vertex<T>* s = g->findVertex(0);
    s->setDist(0);
    MutablePriorityQueue<Vertex<T>> q;
    q.insert(s);

    while( ! q.empty() ) {
        auto v = q.extractMin();
        v->setVisited(true);
        for(auto &e : v->getAdj()) {
            Vertex<T>* w = e->getDest();
            if (!w->isVisited()) {
                auto oldDist = w->getDist();
                if(e->getWeight() < oldDist) {
                    w->setDist(e->getWeight());
                    w->setPath(e);
                    if (oldDist == INF) q.insert(w);
                    else q.decreaseKey(w);
                }
            }
        }
        if (q.empty()){
            for(auto &e : v->getAdj()){
                Vertex<T>* w = e->getDest();
                if (w->getInfo() == 0){
                    w->setPath(e);
                }
            }
        }
    }
}
/*
void createCycle(Graph<int> * g) {
    for (auto v : g->getVertexSet()) v->setVisited(false);
    auto rootVertex = g->findVertex(0);
    stack<Vertex<int>*> dfsStack;
    dfsStack.push(rootVertex);

    while (!dfsStack.empty()) {
        auto currentVertex = dfsStack.top();
        dfsStack.pop();
        currentVertex->setVisited(true);
        for (auto &edge : currentVertex->getAdj()) {
            auto adjacentVertex = edge->getDest();
            if (!adjacentVertex->isVisited()) {
                // Adicione a aresta à lista de arestas do ciclo
                // Aqui você pode armazenar as arestas em um vetor ou realizar outra ação necessária
                // Exemplo: cycleEdges.push_back(edge);

                // Adicione o vértice adjacente à pilha para visitar posteriormente
                dfsStack.push(adjacentVertex);
            }
        }
    }

    // Adicione a última aresta que conecta o último vértice visitado de volta à raiz
    // Isso fecha o ciclo
    // Aqui você pode adicionar a aresta diretamente ao vetor de arestas do ciclo ou realizar outra ação necessária

    // Para adicionar a aresta de volta à raiz, você precisa verificar se a raiz tem uma aresta de volta ao vértice inicial
    auto rootEdges = rootVertex->getAdj();
    for (auto &edge : rootEdges) {
        if (edge->getDest() == rootVertex) {
            // Adicione a aresta de volta à raiz à lista de arestas do ciclo
            // Exemplo: cycleEdges.push_back(edge);
            break;
        }
    }
}
int Value(Graph<int> * g){
    int value = 0;
    auto v = g->findVertex(0);
    do{
        value += v->getPath()->getWeight();
        v = v->getPath()->getOrig();
        cout << v->getInfo() << endl;
    }while (v->getInfo() != 0 && v->getPath() != nullptr);
    return value;
}
int Menu::heuristica(Graph<int> * g){
    prim<int>(g);
    cout << Value(g) ;
}*/

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

int Menu::greedyHeuristica(Graph<int> * g){
    std::vector<Edge<int> *> allEdges;
    for (auto v: g->getVertexSet()) {
        v->cleanConnect();
        for (auto e: v->getAdj()) {
            allEdges.push_back(e);
        }
    }
    sort(allEdges.begin(), allEdges.end(), [](Edge<int> *a, Edge<int> *b) { return a->getWeight() < b->getWeight(); });
    int totalWeight = 0;
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
    for (auto v: g->getVertexSet()) {
        cout << v->getInfo() << " : "<<v->getAdj().size() << endl;
    }
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


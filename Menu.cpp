//
// Created by Alexandre on 03/05/2024.
//

#include <vector>
#include "DataStructures/Graph.h"
#include "Menu.h"
#include <stack>
#include <set>

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
}*/
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
}

template <class T>
void dfsKruskalPath(Vertex<T> *v) {
    v->setVisited(true);
    for (auto e : v->getAdj()) {
        if (e->isSelected() && !e->getDest()->isVisited()) {
            e->getDest()->setPath(e);
            dfsKruskalPath(e->getDest());
        }
    }
}
template <class T>
bool isCycle(Vertex<T> *s, Vertex<T> *t, int size){
    cout << "cycle "<< s->getInfo() << ":" << t->getInfo() << endl;
    if (s == t) return true;
    size--;
    queue<Vertex<int> *> q;
    q.push(s);
    while (size > 1 && !q.empty()){
        s = q.front();
        q.pop();
        cout << s->getInfo() << endl;
        for (auto v : s->getConnects()){
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
        v->setIndegree(0);
        for (auto e: v->getAdj()) {
            if (e->getOrig()->getInfo() < e->getDest()->getInfo()) {
                allEdges.push_back(e);
            }
        }
    }
    sort(allEdges.begin(), allEdges.end(), [](Edge<int> *a, Edge<int> *b) { return a->getWeight() < b->getWeight(); });
    unsigned selectedEdges = 0;
    int totalWeight = 0;
        for (auto e : allEdges) {
            for (auto v: g->getVertexSet()) v->setVisited(false);
                if (e->getOrig()->getIndegree() < 2 && e->getDest()->getIndegree() < 2 && !isCycle(e->getDest(),e->getOrig(), g->getVertexSet().size())) {
                cout << "Connect " << e->getOrig()->getInfo() << ":" << e->getDest()->getInfo() << " , w : " << e->getWeight() << endl;
                totalWeight += e->getWeight();
                e->getOrig()->addConnect(e->getDest());
                e->getDest()->addConnect(e->getOrig());
                e->getOrig()->setIndegree(e->getOrig()->getIndegree()+1);
                e->getDest()->setIndegree(e->getDest()->getIndegree()+1);
                selectedEdges++;
            }
        }
    return totalWeight;
}



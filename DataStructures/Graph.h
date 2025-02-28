
// Original code by Gonçalo Leão
// Updated by DA 2023/2024 Team

#ifndef DA_TP_CLASSES_GRAPH
#define DA_TP_CLASSES_GRAPH

#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include "MutablePriorityQueue.h"
#include "Harvesine.h"
using namespace std;

template <class T>
class Edge;

#define INF std::numeric_limits<double>::max()


/************************* Vertex  **************************/

template <class T>
class Vertex {
public:
    Vertex(T in);
    bool operator<(Vertex<T> & vertex) const; // // required by MutablePriorityQueue

    T getInfo() const;
    std::vector<Edge<T> *> getAdj() const;
    bool isVisited() const;
    bool isProcessing() const;
    void setInfo(T info);
    void setVisited(bool visited);
    void setProcesssing(bool processing);
    void setIndegree(unsigned int indegree);
    void addAdj(Vertex<int> * v, double w);
    void setDist(double dist);
    double getDist() const;
    Edge<int>*  getPath();
    void setPath(Edge<int>* p);
    bool removeEdgeFromConnect(T in);
    double getWeightTo(T info);
    double getWeightToHarvesine(Vertex<T> *v);

    //New
    std::vector<Edge<T> *> getConnects() const;
    void addConnect(Edge<T> *edge);
    void cleanConnect();
    double getLongitude();
    double getLatitude();
    void setLatitude(double l);
    void setLongitude(double l);

    friend class MutablePriorityQueue<Vertex>;
protected:
    T info;                // info node
    std::vector<Edge<T> *> adj;  // outgoing and incoming edges
    double dist = 0;
    // auxiliary fields
    bool visited = false; // used by DFS, BFS, Prim ...
    bool processing = false; // used by isDAG (in addition to the visited attribute)
    Edge<int>* path = nullptr;
    double longitude = 0;
    double latitude = 0;
    int queueIndex = 0; 		// required by MutablePriorityQueue and UFDS

    //New
    std::vector<Edge<T> *> connects;
};

/********************** Edge  ****************************/

template <class T>
class Edge {
public:
    Edge(Vertex<T> *orig, Vertex<T> *dest, double w);
    double getWeight() const;
    // New
    pair<Vertex<int> *,Vertex<int> *> getPair() const;
    Vertex<int> * getVertex(Vertex<int>* v) const;
protected:
    double weight; // edge weight, can also be used for capacity
    pair<Vertex<int> *,Vertex<int> *> vertexs;
};

/********************** Graph  ****************************/

template <class T>
class Graph {
public:
    ~Graph();
    /*
    * Auxiliary function to find a vertex with a given the content.
    */
    Vertex<T> *findVertex(const T &in) const;
    /*
     *  Adds a vertex with a given content or info (in) to a graph (this).
     *  Returns true if successful, and false if a vertex with that content already exists.
     */
    bool addVertex(const T &in);
    bool removeVertex(const T &in);

    /*
     * Adds an edge to a graph (this), given the contents of the source and
     * destination vertices and the edge weight (w).
     * Returns true if successful, and false if the source or destination vertex does not exist.
     */
    bool addEdge(const T &sourc, const T &dest, double w);
    bool removeEdge(const T &source, const T &dest);
    bool addBidirectionalEdge(const T &sourc, const T &dest, double w);

    int getNumVertex() const;
    unordered_map<T, Vertex<T> *> getVertexSet() const;

    std:: vector<T> dfs() const;
    std:: vector<T> dfs(const T & source) const;
    void dfsVisit(Vertex<T> *v,  std::vector<T> & res) const;
    std::vector<T> bfs(const T & source) const;

    bool isDAG() const;
    bool dfsIsDAG(Vertex<T> *v) const;
    std::vector<T> topsort() const;
protected:
    std::unordered_map<T, Vertex<T> *> vertexSet;    // vertex set

    double ** distMatrix = nullptr;   // dist matrix for Floyd-Warshall
    int **pathMatrix = nullptr;   // path matrix for Floyd-Warshall

    /*
     * Finds the index of the vertex with a given content.
     */
    int findVertexIdx(const T &in) const;
};

void deleteMatrix(int **m, int n);
void deleteMatrix(double **m, int n);


/************************* Vertex  **************************/

template <class T>
Vertex<T>::Vertex(T in): info(in) {}
/*
 * Auxiliary function to add an outgoing edge to a vertex (this),
 * with a given destination vertex (d) and edge weight (w).
 */


/*
 * Auxiliary function to remove an outgoing edge (with a given destination (d))
 * from a vertex (this).
 * Returns true if successful, and false if such edge does not exist.
 */


/*
 * Auxiliary function to remove an outgoing edge of a vertex.
 */


template <class T>
bool Vertex<T>::operator<(Vertex<T> & vertex) const {
    return this->dist < vertex.dist;
}

template <class T>
T Vertex<T>::getInfo() const {
    return this->info;
}

template <class T>
std::vector<Edge<T>*> Vertex<T>::getAdj() const {
    return this->adj;
}
template <class T>
std::vector<Edge<T> *> Vertex<T>::getConnects() const {
    return this->connects;
}
template <class T>
void Vertex<T>::addConnect(Edge<T> *edge) {
    this->connects.push_back(edge);
}
template <class T>
void Vertex<T>::cleanConnect(){
    this->connects.clear();
}
template <class T>
double Vertex<T>::getLongitude(){
    return this->longitude;
}
template <class T>
double Vertex<T>::getLatitude(){
    return this->latitude;
}
template <class T>
void Vertex<T>::setLatitude(double l){
    this->latitude = l;
}
template <class T>
void Vertex<T>::setLongitude(double l){
    this->longitude = l;
}

template <class T>
bool Vertex<T>::isVisited() const {
    return this->visited;
}

template <class T>
bool Vertex<T>::isProcessing() const {
    return this->processing;
}

template <class T>
void Vertex<T>::setInfo(T in) {
    this->info = in;
}

template <class T>
void Vertex<T>::setVisited(bool visited) {
    this->visited = visited;
}

template <class T>
void Vertex<T>::setProcesssing(bool processing) {
    this->processing = processing;
}

template <class T>
void Vertex<T>::setIndegree(unsigned int indegree) {
    this->indegree = indegree;
}
template <class T>
void Vertex<T>::addAdj(Vertex<int> * v, double w){
    auto newEdge = new Edge<T>(this, v, w);
    this->adj.push_back(newEdge);
    auto newEdge2 = new Edge<T>(v, this, w);
    v->adj.push_back(newEdge2);
}
template <class T>
double Vertex<T>::getDist() const {
    return this->dist;
}
template <class T>
void Vertex<T>::setDist(double dist) {
    this->dist = dist;
}
template <class T>
Edge<int>* Vertex<T>::getPath(){
    return this->path;
}
template <class T>
void Vertex<T>::setPath(Edge<int>* p){
    this->path = p;
}
template <class T>
bool Vertex<T>::removeEdgeFromConnect(T in) {
    bool removedEdge = false;
    auto it = connects.begin();
    while (it != connects.end()) {
        Edge<T> *edge = *it;
        Vertex<T> *dest = edge->getVertex(this);
        if (dest->getInfo() == in) {
            it = connects.erase(it);
            removedEdge = true; // allows for multiple edges to connect the same pair of vertices (multigraph)
        }
        else {
            it++;
        }
    }
    return removedEdge;
}
template <class T>
double Vertex<T>::getWeightTo(T info){
    for (auto e : this->adj){
        if (e->getVertex(this)->getInfo() == info) return e->getWeight();
    }
    return -1;
}
template <class T>
double Vertex<T>::getWeightToHarvesine(Vertex<T> *v){
    for (auto e : this->adj){
        if (e->getVertex(this) == v) return e->getWeight();
    }
    return Harvesine::haversineDistance(this->latitude, this->longitude, v->getLatitude(), v->getLongitude());
}

/********************** Edge  ****************************/

template <class T>
Edge<T>::Edge(Vertex<T> *orig, Vertex<T> *dest, double w): weight(w) {
    vertexs = {orig, dest};
}

template <class T>
double Edge<T>::getWeight() const {
    return this->weight;
}

template <class T>
pair<Vertex<int> *,Vertex<int> *> Edge<T>::getPair() const{
    return vertexs;
}
template <class T>
Vertex<int> * Edge<T>::getVertex(Vertex<int>* v) const{
    if (vertexs.first == v) return vertexs.second;
    else return vertexs.first;
}

/********************** Graph  ****************************/

template <class T>
int Graph<T>::getNumVertex() const {
    return vertexSet.size();
}

template <class T>
unordered_map<T, Vertex<T> *> Graph<T>::getVertexSet() const {
    return vertexSet;
}

/*
 * Auxiliary function to find a vertex with a given content.
 */
template <class T>
Vertex<T> * Graph<T>::findVertex(const T &in) const {
    auto r = vertexSet.find(in);
    if (r == vertexSet.end()) return nullptr;
    else return r->second;
}

/*
 * Finds the index of the vertex with a given content.
 */
template <class T>
int Graph<T>::findVertexIdx(const T &in) const {
    for (unsigned i = 0; i < vertexSet.size(); i++)
        if (vertexSet[i]->getInfo() == in)
            return i;
    return -1;
}
/*
 *  Adds a vertex with a given content or info (in) to a graph (this).
 *  Returns true if successful, and false if a vertex with that content already exists.
 */
template <class T>
bool Graph<T>::addVertex(const T &in) {
    if (findVertex(in) != nullptr)
        return false;
    vertexSet.insert({in, new Vertex<T>(in)});
    return true;
}


/*
 *  Removes a vertex with a given content (in) from a graph (this), and
 *  all outgoing and incoming edges.
 *  Returns true if successful, and false if such vertex does not exist.
 */
/*
template <class T>
bool Graph<T>::removeVertex(const T &in) {
    for (auto it = vertexSet.begin(); it != vertexSet.end(); it++) {
        if ((*it)->getInfo() == in) {
            auto v = *it;
            v->removeOutgoingEdges();
            for (auto u : vertexSet) {
                u->removeEdge(v->getInfo());
            }
            vertexSet.erase(it);
            delete v;
            return true;
        }
    }
    return false;
}*/

/*
 * Adds an edge to a graph (this), given the contents of the source and
 * destination vertices and the edge weight (w).
 * Returns true if successful, and false if the source or destination vertex does not exist.
 */
template <class T>
bool Graph<T>::addEdge(const T &sourc, const T &dest, double w) {
    auto v1 = findVertex(sourc);
    auto v2 = findVertex(dest);
    if (v1 == nullptr || v2 == nullptr)
        return false;
    v1->addAdj(v2, w);
    return true;
}

/*
 * Removes an edge from a graph (this).
 * The edge is identified by the source (sourc) and destination (dest) contents.
 * Returns true if successful, and false if such edge does not exist.
 */
template <class T>
bool Graph<T>::removeEdge(const T &sourc, const T &dest) {
    Vertex<T> * srcVertex = findVertex(sourc);
    Vertex<T> * endVertex = findVertex(dest);
    if (srcVertex == nullptr || endVertex == nullptr) {
        return false;
    }
    bool removedEdge = false;
    auto it = srcVertex->getAdj().begin();
    cout << (*it)->getVertex(srcVertex)->getInfo() << endl;
    while (it != srcVertex->getAdj().end()) {
        Vertex<T> *d = (*it)->getVertex(srcVertex);
        if (d->getInfo() == dest) {
            for (auto k : srcVertex->getAdj()) cout << k->getVertex(srcVertex)->getInfo() << endl;
            srcVertex->getAdj().erase(it);
            removedEdge = true; // allows for multiple edges to connect the same pair of vertices (multigraph)
            break;
        }
        else {
            it++;
        }
    }
    if (!removedEdge) return false;
    removedEdge = false;
    it = endVertex->getAdj().begin();
    while (it != endVertex->getAdj().end()) {
        Edge<T> *edge = *it;

        Vertex<T> *d = edge->getVertex(endVertex);
        if (d->getInfo() == sourc) {
            it = endVertex->getAdj().erase(it);
            removedEdge = true; // allows for multiple edges to connect the same pair of vertices (multigraph)
        }
        else {
            it++;
        }
    }
    return removedEdge;
}

template <class T>
bool Graph<T>::addBidirectionalEdge(const T &sourc, const T &dest, double w) {
    auto v1 = findVertex(sourc);
    auto v2 = findVertex(dest);
    if (v1 == nullptr || v2 == nullptr)
        return false;
    auto e1 = v1->addEdge(v2, w);
    auto e2 = v2->addEdge(v1, w);
    e1->setReverse(e2);
    e2->setReverse(e1);
    return true;
}

/****************** DFS ********************/

/*
 * Performs a depth-first search (dfs) traversal in a graph (this).
 * Returns a vector with the contents of the vertices by dfs order.
 */
template <class T>
std::vector<T> Graph<T>::dfs() const {
    std::vector<T> res;
    for (auto v : vertexSet)
        v->setVisited(false);
    for (auto v : vertexSet)
        if (!v->isVisited())
            dfsVisit(v, res);
    return res;
}

/*
 * Performs a depth-first search (dfs) in a graph (this) from the source node.
 * Returns a vector with the contents of the vertices by dfs order.
 */
template <class T>
std::vector<T> Graph<T>::dfs(const T & source) const {
    std::vector<int> res;
    // Get the source vertex
    auto s = findVertex(source);
    if (s == nullptr) {
        return res;
    }
    // Set that no vertex has been visited yet
    for (auto v : vertexSet) {
        v->setVisited(false);
    }
    // Perform the actual DFS using recursion
    dfsVisit(s, res);

    return res;
}

/*
 * Auxiliary function that visits a vertex (v) and its adjacent, recursively.
 * Updates a parameter with the list of visited node contents.
 */
template <class T>
void Graph<T>::dfsVisit(Vertex<T> *v, std::vector<T> & res) const {
    v->setVisited(true);
    res.push_back(v->getInfo());
    for (auto & e : v->getAdj()) {
        auto w = e->getDest();
        if (!w->isVisited()) {
            dfsVisit(w, res);
        }
    }
}

/****************** BFS ********************/
/*
 * Performs a breadth-first search (bfs) in a graph (this), starting
 * from the vertex with the given source contents (source).
 * Returns a vector with the contents of the vertices by bfs order.
 */
template <class T>
std::vector<T> Graph<T>::bfs(const T & source) const {
    std::vector<int> res;
    // Get the source vertex
    auto s = findVertex(source);
    if (s == nullptr) {
        return res;
    }

    // Set that no vertex has been visited yet
    for (auto v : vertexSet) {
        v->setVisited(false);
    }

    // Perform the actual BFS using a queue
    std::queue<Vertex<T> *> q;
    q.push(s);
    s->setVisited(true);
    while (!q.empty()) {
        auto v = q.front();
        q.pop();
        res.push_back(v->getInfo());
        for (auto & e : v->getAdj()) {
            auto w = e->getDest();
            if ( ! w->isVisited()) {
                q.push(w);
                w->setVisited(true);
            }
        }
    }
    return res;
}

/****************** isDAG  ********************/
/*
 * Performs a depth-first search in a graph (this), to determine if the graph
 * is acyclic (acyclic directed graph or DAG).
 * During the search, a cycle is found if an edge connects to a vertex
 * that is being processed in the stack of recursive calls (see theoretical classes).
 * Returns true if the graph is acyclic, and false otherwise.
 */

template <class T>
bool Graph<T>::isDAG() const {
    for (auto v : vertexSet) {
        v->setVisited(false);
        v->setProcesssing(false);
    }
    for (auto v : vertexSet) {
        if (! v->isVisited()) {
            if ( ! dfsIsDAG(v) ) return false;
        }
    }
    return true;
}

/**
 * Auxiliary function that visits a vertex (v) and its adjacent, recursively.
 * Returns false (not acyclic) if an edge to a vertex in the stack is found.
 */
template <class T>
bool Graph<T>::dfsIsDAG(Vertex<T> *v) const {
    v->setVisited(true);
    v->setProcesssing(true);
    for (auto e : v->getAdj()) {
        auto w = e->getDest();
        if (w->isProcessing()) return false;
        if (! w->isVisited()) {
            if (! dfsIsDAG(w)) return false;
        }
    }
    v->setProcesssing(false);
    return true;
}

/****************** toposort ********************/
//=============================================================================
// Exercise 1: Topological Sorting
//=============================================================================
// TODO
/*
 * Performs a topological sorting of the vertices of a graph (this).
 * Returns a vector with the contents of the vertices by topological order.
 * If the graph has cycles, returns an empty vector.
 * Follows the algorithm described in theoretical classes.
 */

template<class T>
std::vector<T> Graph<T>::topsort() const {
    std::vector<int> res;

    for (auto v : vertexSet) {
        v->setIndegree(0);
    }
    for (auto v : vertexSet) {
        for (auto e : v->getAdj()) {
            unsigned int indegree = e->getDest()->getIndegree();
            e->getDest()->setIndegree(indegree + 1);
        }
    }

    std::queue<Vertex<T> *> q;
    for (auto v : vertexSet) {
        if (v->getIndegree() == 0) {
            q.push(v);
        }
    }

    while( !q.empty() ) {
        Vertex<T> * v = q.front();
        q.pop();
        res.push_back(v->getInfo());
        for(auto e : v->getAdj()) {
            auto w = e->getDest();
            w->setIndegree(w->getIndegree() - 1);
            if(w->getIndegree() == 0) {
                q.push(w);
            }
        }
    }

    if ( res.size() != vertexSet.size() ) {
        //std::cout << "Impossible topological ordering!" << std::endl;
        res.clear();
        return res;
    }

    return res;
}

inline void deleteMatrix(int **m, int n) {
    if (m != nullptr) {
        for (int i = 0; i < n; i++)
            if (m[i] != nullptr)
                delete [] m[i];
        delete [] m;
    }
}

inline void deleteMatrix(double **m, int n) {
    if (m != nullptr) {
        for (int i = 0; i < n; i++)
            if (m[i] != nullptr)
                delete [] m[i];
        delete [] m;
    }
}

template <class T>
Graph<T>::~Graph() {
    deleteMatrix(distMatrix, vertexSet.size());
    deleteMatrix(pathMatrix, vertexSet.size());
}


#endif /* DA_TP_CLASSES_GRAPH */

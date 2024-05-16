
#include <vector>
#include "DataStructures/Graph.h"
#include "Menu.h"
#include <random>
#include <stack>
#include <set>
#include <cmath>
#include <unordered_map>

double pi=3.14159265358979323846;
double earthradius=6371000; //meters
/**
 * @brief Converts degrees to radians.
 * @param coord Value of the coordinate in degrees.
 * @return Value of the coordinate in radians.
 */
double toRadians(double coord){
    return coord*pi/180.0;
}

/**
 * @brief Gives the Haversine distance between two coordinates.
 * @param lat1 Latitude of the first point in degrees.
 * @param lon1 Longitude of the first point in degrees.
 * @param lat2 Latitude of the second point in degrees.
 * @param lon2 Longitude of the second point in degrees.
 * @return The Haversine distance between the two points in meters.
 */
double haversineDistance(double lat1, double lon1, double lat2, double lon2){
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


/**
 * @brief Shows if exists a cycle between two vertices in the graph.
 * This function does a breadth-first search (BFS) starting in vertex s to see if exists a
 * path from vertex s to t. If that path exists there is a cycle between vertices s and t.
 * @tparam T The type of the vertices.
 * @param s Pointer to the source vertex.
 * @param t Pointer to the target vertex.
 * @param size Number of vertices in the graph.
 * @return It returns true if exists a cycle between vertices s and t. False if it doesn't exist.
 */
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
            if (v1->getConnects().size() >= 2 || v2->getConnects().size() >= 2) continue;
            for (auto v: g->getVertexSet()) v->setVisited(false);
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


double Menu::randomSwap(Graph<int>* g, vector<int>& minPath, double minDist) {
    double minDistS = minDist;
    int size = g->getNumVertex()-1;

    random_device rd; mt19937 gen(rd());
    uniform_int_distribution<int> dist(1, size-1);
    int pos1 = dist(gen), pos2 = dist(gen);
    while (pos1 == pos2) pos2 = dist(gen);

    int n1 = minPath[pos1], n2 = minPath[pos2];
    if (abs(pos1-pos2) == 1) {
        if (pos2 < pos1) {
            //Trocar a ordem para crescente
            int temp = pos1; pos1 = pos2; pos2 = temp;
            temp = n1; n1 = n2; n2 = temp;

            int bef1 = minPath[pos1 - 1];
            int aft2 = minPath[pos2 + 1];

            for (auto e: g->findVertex(n1)->getAdj()) {
                auto v = e->getPair().second;
                double w = e->getWeight();

                if (v->getInfo() == bef1)  {minDistS -= w;}
                if (v->getInfo() == aft2)  {minDistS += w;}
            }

            for (auto e: g->findVertex(n2)->getAdj()) {
                auto v = e->getPair().second;
                double w = e->getWeight();

                if (v->getInfo() == aft2)  {minDistS -= w;}
                if (v->getInfo() == bef1)  {minDistS += w;}
            }
        }

    } else {
        int bef1 = minPath[pos1 - 1];
        int bef2 = minPath[pos2 - 1];
        int aft1 = minPath[pos1 + 1];
        int aft2 = minPath[pos2 + 1];

        for (auto e: g->findVertex(n1)->getAdj()) {
            auto v = e->getPair().second;
            double w = e->getWeight();

            if (v->getInfo() == bef1 || v->getInfo() == aft1)  {minDistS -= w;}
            if (v->getInfo() == bef2 || v->getInfo() == aft2)  {minDistS += w;}
        }

        for (auto e: g->findVertex(n2)->getAdj()) {
            auto v = e->getPair().second;
            double w = e->getWeight();

            if (v->getInfo() == bef2 || v->getInfo() == aft2)  {minDistS -= w;}
            if (v->getInfo() == bef1 || v->getInfo() == aft1)  {minDistS += w;}
        }
    }

    if (minDistS < minDist) {
        minPath[pos1] = n2; minPath[pos2] = n1;
        return minDistS;
    }
    return minDist;
}


double distancePath(Graph<int>* g, vector<int>& path) {
    double distance = 0;
    int n = path.size();
    for (int i = 0; i < n - 1; i++) {
        for (auto edge: g->findVertex(path[i])->getAdj()) {
            if (edge->getPair().second->getInfo() == path[i+1])
                distance += edge->getWeight();
        }
    }

    for (auto edge: g->findVertex(path[n - 1])->getAdj()) {
        if (edge->getPair().second->getInfo() == path[0])
            distance += edge->getWeight();
    }

    return distance;
}


void Menu::twoOpt(Graph<int>* g, vector<int>& minPath, double& minDist) {
    int n = g->getNumVertex();
    bool improve = true;

    while (improve) {
        improve = false;
        for (int i = 1; i < n - 2; ++i) {
            for (int j = i + 1; j < n - 1; ++j) {
                swap(minPath[i], minPath[j]);
                double newDist = distancePath(g, minPath);
                if (newDist < minDist) {
                    improve = true;
                    minDist = newDist;
                } else {
                    swap(minPath[i], minPath[j]);
                }
            }
        }
    }
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
    return 0;
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
                double dist = calculateDistance(graph, currentPath[pathSize - 1], i);
                if (dist < minDistance) {
                    nextVert->setVisited(true);
                    nextVert->setDist(dist);
                    currentPath.push_back(i);

                    backtrack(graph, currentPath, bestPath, minDistance);

                    nextVert->setVisited(false);
                    currentPath.pop_back();
                }
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
        if (v->getPath() == nullptr ) continue;
        for (auto e : v->getAdj()){
            if (e == v->getPath() || (e->getPair().first == v->getPath()->getPair().second) && (e->getPair().second == v->getPath()->getPair().first)){
                v->addConnect(e);
                e->getVertex(v)->addConnect(e);
            }
        }
        v->setDist(0);
    }
    while (!oddVertex.empty()) {
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
    }
    for (auto v : g->getVertexSet()){
        //cout <<" !! "<<v->getInfo() << endl;
        for (auto e : v->getConnects()){
            //cout << e->getVertex(v)->getInfo() << endl;
        }
    }
}

template <class T>
void dfs(Vertex<T>* v, vector<Edge<T>*>& eulerianCircuit) {
    while (!v->getConnects().empty()) {
        Edge<T>* e = v->getConnects().front();
        Vertex<T>* u = e->getVertex(v);
        v->removeEdgeFromConnect(u->getInfo());
        u->removeEdgeFromConnect(v->getInfo());
        eulerianCircuit.push_back(e);
        dfs(u, eulerianCircuit);
    }
}

template <class T>
void findEulerianCircuit(Vertex<T> *v, vector<Edge<T>*>& eulerianCircuit) {
    dfs(v, eulerianCircuit);
}
template <class T>
void Tsp(vector<Edge<int>*> &path, double &cost, Graph<T> *g){
    cost = 0;

    vector<Edge<int>*>::iterator it = path.begin();

    int actualV = 0;
    int lastV = 0;
    int carryV = 0;

    while(it != path.end()){
        Vertex<int> * v1 = (*it)->getPair().first;
        Vertex<int> * v2 = (*it)->getPair().second;
        if (v1->getInfo() == actualV || v1->getInfo() == carryV){
            if (v2->isVisited()){
                carryV = v2->getInfo();
            }else actualV = v2->getInfo();
        }else if (v2->getInfo() == actualV || v2->getInfo() == carryV){
            if (v1->isVisited()){
                carryV = v2->getInfo();
            }else actualV = v1->getInfo();
        }else{ // Back edge
            if (v1->isVisited()){
                actualV = v2->getInfo();
            }else{
                actualV = v1->getInfo();
            }
        }
        if (actualV != lastV){
            //cout << "de " << lastV << " para " << actualV << endl;
            double v = g->findVertex(actualV)->getWeightTo(lastV);
            if (v == -1)
                cost += haversineDistance(g->findVertex(actualV)->getLatitude(), g->findVertex(actualV)->getLongitude(), g->findVertex(lastV)->getLatitude(), g->findVertex(lastV)->getLongitude());
            else cost += v;
            lastV = actualV;
        }
        v1->setVisited(true);
        v2->setVisited(true);
        it++;
    }
    //cout << "de " << actualV << " para " << 0 << endl;
    double v = g->findVertex(0)->getWeightTo(actualV);
    if (v == -1) {
        cost += haversineDistance(g->findVertex(0)->getLatitude(), g->findVertex(0)->getLongitude(), g->findVertex(actualV)->getLatitude(), g->findVertex(actualV)->getLongitude());
    }else cost += v;

}

double Menu::Cristofides(Graph<int> * g, vector<int>& minPath){
    prim(g);
    BestMatch(g);
    vector<Edge<int>*> eulerian;
    findEulerianCircuit(g->findVertex(0), eulerian);
    //for (auto e : eulerian)  cout << e->getPair().first->getInfo() << " : " << e->getPair().second->getInfo() << endl;
    for (auto v : g->getVertexSet()) v->setVisited(false);
    double cost;
    Tsp(eulerian, cost, g);
    return cost;
}

 pair<double,int> Menu::nearestNeighborTSP(Graph<int> *graph, vector<int>& minPath, int inicialVertex){
    stack<Vertex<int>*> s;
    double res = 0;
    for (auto v : graph->getVertexSet()) v->setVisited(false);
    int numVertices = graph->getNumVertex();
    minPath.push_back(inicialVertex);
    auto v = graph->findVertex(inicialVertex);
    s.push(v);
    for (int i = 0; i < numVertices - 1; i++) {
        v = s.top();
        if (v->getInfo() == inicialVertex && i != 0) break; // Break case
        int nearestNeighbor = -1;
        double minDistance = numeric_limits<double>::infinity();
        v->setVisited(true);
        for (auto e : v->getAdj()){
            auto d = e->getVertex(v);
            if(!d->isVisited()){
                double distance = e->getWeight();
                if (distance < minDistance) {
                    minDistance = distance;
                    nearestNeighbor = d->getInfo();
                }
            }
        }

        cout << v->getInfo() << "!" << nearestNeighbor << endl;
        if (nearestNeighbor == -1) { // NO edge available
            auto k =  v->getWeightTo(inicialVertex);
            if (k != -1) return {res + k,s.size()};
            s.pop();
            v->setVisited(true);
            res -= v->getWeightTo(s.top()->getInfo());
            i -= 2;
            continue;
        }
        res += minDistance;
        minPath.push_back(nearestNeighbor);
        s.push(graph->findVertex(nearestNeighbor));
        if (i == numVertices - 2){
            auto k =  graph->findVertex(nearestNeighbor)->getWeightTo(inicialVertex);
            if (k != -1) return {res + k,s.size()};
            s.pop();
            v->setVisited(true);
            res -= v->getWeightTo(s.top()->getInfo());
            i -= 2;
            continue;
        }
    }
    return {0,0};
}


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

double Menu::triangularApproximation(Graph<int>* g, vector<int>& minPath){
    double r=0;
    vector p=prim(g);
    for(auto i=0; i<p.size(); i++){
        r+=p[i]->getDist();
        //cout<<p[i]->getDist()<<endl;
        minPath.push_back(p[i]->getInfo());
    }

    Vertex<int> * ultimo=p[p.size()-1];

    r+=ultimo->getWeightTo(p[p.size()-2]->getInfo());
    //cout<<ultimo->getWeightTo(p[p.size()-2]->getInfo())<<endl;

    return r;
}




#include "Menu.h"



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
    for (auto pair: g->getVertexSet()) {
        auto v = pair.second;
        v->cleanConnect();
        for (auto e: v->getAdj()) {
            if (v->getInfo() < e->getVertex(v)->getInfo()) allEdges.push_back(e);
        }
    }
    int count = 0;
    sort(allEdges.begin(), allEdges.end(), [](Edge<int> *a, Edge<int> *b) { return a->getWeight() < b->getWeight(); });
    double totalWeight = 0;

        for (auto e : allEdges) {
            auto v1 = e->getPair().first;
            auto v2 = e->getPair().second;
            if (v1->getConnects().size() >= 2 || v2->getConnects().size() >= 2) continue;
            for (auto pair: g->getVertexSet()){
                auto v = pair.second;
                v->setVisited(false);
            }
            if (v1->getConnects().size() == 0 || v2->getConnects().size() == 0
                || !isCycle(v2,v1, g->getVertexSet().size())) {
                    totalWeight += e->getWeight();
                    v1->addConnect(e);
                    v2->addConnect(e);
                    count++;
                }
            if (count >= g->getNumVertex()) {break;}

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

            minDistS -= g->findVertex(n1)->getWeightToHarvesine(g->findVertex(bef1));
            minDistS += g->findVertex(n1)->getWeightToHarvesine(g->findVertex(aft2));

            minDistS -= g->findVertex(n2)->getWeightToHarvesine(g->findVertex(aft2));
            minDistS += g->findVertex(n2)->getWeightToHarvesine(g->findVertex(bef1));
        }

    } else {
        int bef1 = minPath[pos1 - 1];
        int bef2 = minPath[pos2 - 1];
        int aft1 = minPath[pos1 + 1];
        int aft2 = minPath[pos2 + 1];


        minDistS -= g->findVertex(n1)->getWeightToHarvesine(g->findVertex(bef1));
        minDistS -= g->findVertex(n1)->getWeightToHarvesine(g->findVertex(aft1));
        minDistS += g->findVertex(n1)->getWeightToHarvesine(g->findVertex(bef2));
        minDistS += g->findVertex(n1)->getWeightToHarvesine(g->findVertex(aft2));

        minDistS -= g->findVertex(n2)->getWeightToHarvesine(g->findVertex(bef2));
        minDistS -= g->findVertex(n2)->getWeightToHarvesine(g->findVertex(aft2));
        minDistS += g->findVertex(n2)->getWeightToHarvesine(g->findVertex(bef1));
        minDistS += g->findVertex(n2)->getWeightToHarvesine(g->findVertex(aft1));
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
        distance += g->findVertex(path[i])->getWeightToHarvesine(g->findVertex(path[i+1]));
    }

    // Aresta de volta ao inicial
    distance += g->findVertex(path[n - 1])->getWeightToHarvesine(g->findVertex(path[0]));

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



/**
 * @brief Sees if there is a path between two vertices in the graph.
 * This function sees if there is a path from a source to a destination vertex.
 * @param graph Graph.
 * @param infoOrig Source vertex info.
 * @param infoDest Destination vertex info.
 * @return It is true if there is a path between the source and destination vertices.
 * @details Complexity-> O(n), n is the average number of edges in each vertex.
 */
bool existsPath(Graph<int>& graph, int infoOrig, int infoDest) {
    for (auto edge: graph.findVertex(infoOrig)->getAdj())
        if (edge->getVertex(graph.findVertex(infoOrig))->getInfo() == infoDest)
            return true;
    return false;
}

/**
 * @brief Shows the distance between two vertices.
 * This function calculates the distance between two vertices in the graph.
 * @param graph Graph.
 * @param infoLast Source vertex info.
 * @param infoNext Destination vertex info.
 * @return The distance of the source to the destination vertices if exists a direct edge between them and 0 if it doesn't exist.
 * @details Complexity-> O(n), n is the number of edges adjacent to the vertex with the information infoLast.
 */
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
    for (auto pair: graph.getVertexSet()) {
        auto vertex = pair.second;
        vertex->setVisited(false);
        vertex->setDist(0);
    }

    graph.findVertex(0)->setVisited(true);
    vector<int> currentPath = {0};     // Start from vertex 0 *
    double minDistance = numeric_limits<double>::max();

    backtrack(graph, currentPath, minPath, minDistance);
    return minDistance;
}
/*

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
            //if (v == -1)
               // cost += haversineDistance(g->findVertex(actualV)->getLatitude(), g->findVertex(actualV)->getLongitude(), g->findVertex(lastV)->getLatitude(), g->findVertex(lastV)->getLongitude());
           // else cost += v;
           // lastV = actualV;
        }
        v1->setVisited(true);
        v2->setVisited(true);
        it++;
    }
    //cout << "de " << actualV << " para " << 0 << endl;
    double v = g->findVertex(0)->getWeightTo(actualV);
    if (v == -1) {
       // cost += haversineDistance(g->findVertex(0)->getLatitude(), g->findVertex(0)->getLongitude(), g->findVertex(actualV)->getLatitude(), g->findVertex(actualV)->getLongitude());
    }else cost += v;

}
*/

 pair<double,int> Menu::nearestNeighborTSP(Graph<int> *graph, vector<int>& minPath, int inicialVertex){
    stack<Vertex<int>*> s;
    double res = 0;
    for (auto pair : graph->getVertexSet()){
        auto v = pair.second;
        v->setVisited(false);
    }
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

        //cout << v->getInfo() << "!" << nearestNeighbor << endl;
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

template <class T>
void prim(Graph<T> * g) {
     stack<Edge<T> *> que;
    for(auto pair : g->getVertexSet()) {
        auto v = pair.second;
        v->setDist(INF);
        v->setPath(nullptr);
        v->setVisited(false);
    }
    Vertex<T>* s =g->findVertex(0);
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
                    que.push(e);
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
    while (!que.empty()){
        auto e = que.top();
        que.pop();
        if (e->getPair().first->getPath() != e && e->getPair().second->getPath() != e) continue;

        e->getPair().first->addConnect(e);
        e->getPair().second->addConnect(e);
    }
}

template <class T>
void dfs(Vertex<T>* v, vector<int>& eulerianCircuit) {
    v->setVisited(true);
    eulerianCircuit.push_back(v->getInfo());
    for (Edge<T>* e : v->getConnects()) {
        Vertex<T>* u = e->getVertex(v);
        if (!u->isVisited()) {
            dfs(u, eulerianCircuit);
        }
    }
}

double Menu::triangularApproximation(Graph<int>* g, vector<int>& minPath){
    double r=0;
    prim(g);
    for (auto p : g->getVertexSet()) p.second->setVisited(false);
    dfs(g->findVertex(0), minPath);
    int lastV = -1;
    for (auto k : minPath){
        if (lastV != -1){
            r += g->findVertex(lastV)->getWeightToHarvesine(g->findVertex(k));
            cout << lastV << "," << k << " : " << g->findVertex(lastV)->getWeightTo(k) << endl;
        }
        lastV = k;
    }
    r += g->findVertex(lastV)->getWeightToHarvesine(g->findVertex(minPath[0]));
    return r;
}


double Menu::simulatedAnnealing(Graph<int>* graph, vector<int>& minPath) {
    random_device rd; mt19937 generator(rd());
    uniform_real_distribution<double> distribution(0.0, 1.0);
    double temperature = 1, coolRate = 0.85;

    // 1 - Initial solution for path
    double minDist = greedyHeuristica(graph, minPath);

    // 2 - Loop for temperature
    while (temperature > 0.1) {
        // 3 - New solution for path : 2-opt
        vector<int> newPath = minPath; double newDist = minDist;
        twoOpt(graph, newPath, newDist);

        // 4 - Check probability
        double deltaDist = minDist - newDist;
        bool prob = exp(deltaDist / temperature) > distribution(generator);
        if ((newDist < minDist) || prob) {
            minPath = newPath;
            minDist = newDist;
        }

        // 5 - Update temperature
        temperature *= coolRate;
    }

    return minDist;
}
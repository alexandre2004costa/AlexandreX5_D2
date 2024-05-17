
#ifndef PROJETO2_MENU_H
#define PROJETO2_MENU_H

#include <set>
#include "DataStructures/Graph.h"
#include <unordered_set>

class Menu {

public:
    static double Cristofides(Graph<int> * g, vector<int>& minPath);
    static double greedyHeuristica(Graph<int> * g, set<int>& minPath);
    static double randomSwap(Graph<int> * g, vector<int>& minPath, double minDistance);
    static void twoOpt(Graph<int>* g, vector<int>& path, double& dist);
    static double Backtracking(Graph<int>& graph, vector<int>& minPath);
    static pair<double,int> nearestNeighborTSP(Graph<int> *graph, vector<int>& minPath, int inicialVertex);
    static double haversineDistance(double lat1, double lon1, double lat2, double lon2);
    static double toRadians(double coord);
    static vector<Vertex<int>*> prim(Graph<int> * g);
    static double triangularApproximation(Graph<int>* graph, vector<int>& minPath);
    static bool existsInVector(int info);
    static void harvesineEdges(Graph<int> * g);


};


#endif //PROJETO2_MENU_H

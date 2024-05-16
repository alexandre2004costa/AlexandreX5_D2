
#ifndef PROJETO2_MENU_H
#define PROJETO2_MENU_H

#include "DataStructures/Graph.h"
#include <unordered_set>

class Menu {

public:
    static double Cristofides(Graph<int> * g, vector<int>& minPath);
    static double greedyHeuristica(Graph<int> * g, vector<int>& minPath);
    static double randomSwap(Graph<int> * g, vector<int>& minPath, double minDistance);
    static void twoOpt(Graph<int>* g, vector<int>& path, double& dist);
    static double Backtracking(Graph<int>& graph, vector<int>& minPath);
    static long double nearestNeighborTSP(Graph<int> *graph, vector<int>& minPath, int inicialVertex);
    double haversineDistance(double lat1, double lon1, double lat2, double lon2);
    double toRadians(double coord);
    static vector<Vertex<int>*> prim(Graph<int> * g);
    static double triangularApproximation(Graph<int>* graph, vector<int>& minPath);
    static bool existsInVector(int info);

};


#endif //PROJETO2_MENU_H

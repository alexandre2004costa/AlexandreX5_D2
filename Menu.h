
#ifndef PROJETO2_MENU_H
#define PROJETO2_MENU_H

#include <vector>
#include "DataStructures/Graph.h"
#include <random>
#include <stack>
#include <cmath>

class Menu {

public:
    static double Cristofides(Graph<int> * g, vector<int>& minPath);
    static double greedyHeuristica(Graph<int> * g, vector<int>& minPath);
    static double randomSwap(Graph<int> * g, vector<int>& minPath, double minDistance);
    static void twoOpt(Graph<int>* g, vector<int>& path, double& dist);
    static double Backtracking(Graph<int>& graph, vector<int>& minPath);
    static pair<double,int> nearestNeighborTSP(Graph<int> *graph, vector<int>& minPath, int inicialVertex);
    static double triangularApproximation(Graph<int>* graph, vector<int>& minPath);
    static double simulatedAnnealing(Graph<int> *graph, vector<int> &minPath);
};


#endif //PROJETO2_MENU_H


#ifndef PROJETO2_MENU_H
#define PROJETO2_MENU_H

#include "DataStructures/Graph.h"

class Menu {

public:
    static double greedyHeuristica(Graph<int> * g, vector<int>& minPath);
    static double Backtracking(Graph<int>& graph, vector<int>& minPath);
    double haversineDistance(double lat1, double lon1, double lat2, double lon2);
    double toRadians(double coord);

};


#endif //PROJETO2_MENU_H

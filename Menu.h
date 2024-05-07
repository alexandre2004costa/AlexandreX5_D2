
#ifndef PROJETO2_MENU_H
#define PROJETO2_MENU_H

#include "DataStructures/Graph.h"

class Menu {

public:
    static double greedyHeuristica(Graph<int> * g, vector<int>& minPath);
    static double Backtracking(Graph<int>& graph, vector<int>& minPath);

};


#endif //PROJETO2_MENU_H

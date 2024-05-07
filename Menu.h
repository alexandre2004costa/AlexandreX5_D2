
#ifndef PROJETO2_MENU_H
#define PROJETO2_MENU_H

#include "DataStructures/Graph.h"

class Menu {

public:
    int heuristica(Graph<int> * g);
    int greedyHeuristica(Graph<int> * g);
    static double Backtracking(Graph<int>& graph, vector<int>& minPath);

};


#endif //PROJETO2_MENU_H

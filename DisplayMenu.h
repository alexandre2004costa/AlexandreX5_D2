
#ifndef PROJETO2_DISPLAYMENU_H
#define PROJETO2_DISPLAYMENU_H

#include <iostream>
#include <chrono>
#include "Data.h"
#include "Menu.h"
#include "DataStructures/Graph.h"
using namespace std;

class DisplayMenu {
    private:
        Graph<int>* graph;
    public:
        int CloseMenu();
        void SelectGraphType();
        void SelectGraphToy();
        void SelectGraphFully();
        void SelectGraphReal();
        void SelectGraphConnect(string nodeFile);
        void Base();
        void ShowResults(int option, double minDist, vector<int> minPath, chrono::duration<double> time, int missedNodes = 0);
        void askContinue();
        void chooseStartingNode();
};


#endif //PROJETO2_DISPLAYMENU_H

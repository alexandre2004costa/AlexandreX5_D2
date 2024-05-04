
#ifndef PROJETO2_DISPLAYMENU_H
#define PROJETO2_DISPLAYMENU_H

#include <iostream>
#include "Data.h"
#include "Menu.h"
#include "DataStructures/Graph.h"
using namespace std;

class DisplayMenu {
    private:
        Graph<int>* graph;
    public:
        void OpenMenu();
        int CloseMenu();
        void Base();
        void ShowResults(int option, double minDist, vector<int> minPath);
        void askContinue();
};


#endif //PROJETO2_DISPLAYMENU_H

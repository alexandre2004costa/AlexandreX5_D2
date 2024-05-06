#include <iostream>

#include "Data.h"
#include "DisplayMenu.h"
#include "Menu.h"

int main() {

    Graph<int> graph;
    Data::loadConnected(&graph, "edges_900.csv");

    /*DisplayMenu displayMenu = DisplayMenu();
    displayMenu.Base();*/

    Menu menu = Menu();
    cout << menu.greedyHeuristica(&graph);
    return 0;
}

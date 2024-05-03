#include <iostream>

#include "Data.h"
#include "DisplayMenu.h"

int main() {

    //Graph<int> graph;
    //Data::loadConnected(&graph, "shipping.csv");

    DisplayMenu displayMenu = DisplayMenu();
    displayMenu.Base();

    return 0;
}


#include "DisplayMenu.h"
#include "Menu.h"
#include <chrono>

int main() {
  
    /*DisplayMenu displayMenu = DisplayMenu();
    displayMenu.OpenMenu();*/
    Graph<int>* graph = new Graph<int>();
    Data::loadGraph(graph, "stadiums.csv", true);
    vector<int> min;
    Menu::Cristofides(graph, min);


    return 0;
}

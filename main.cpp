
#include "DisplayMenu.h"
#include "Menu.h"
#include <chrono>

int main() {

    /*DisplayMenu displayMenu = DisplayMenu();
    displayMenu.OpenMenu();*/

    Graph<int>* graph = new Graph<int>();
    vector<int> min;
    Data::loadGraph(graph, "shipping.csv", true);
    cout << "lido " << endl;
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i <= 15 ; i++){
        auto r1 =  Menu::nearestNeighborTSP(graph, min, i);
        cout << i <<" : "<< r1.second << " , " << r1.first << endl;
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    return 0;
}

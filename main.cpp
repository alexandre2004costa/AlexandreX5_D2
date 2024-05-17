
#include "DisplayMenu.h"
#include "Menu.h"
#include <chrono>
#include <set>

int main() {

    /*DisplayMenu displayMenu = DisplayMenu();
    displayMenu.OpenMenu();*/

    Graph<int>* graph = new Graph<int>();
    vector<int> min;
    Data::loadGraph(graph, "edges.csv", true);
    //Data::loadNodesInfo(graph, "nodes2.csv");
    cout << "lido " << endl;
    auto start = std::chrono::high_resolution_clock::now();
    //cout << Menu::greedyHeuristica(graph, min) << ","<< min.size() << endl;
    for (int i = 0; i <= 15 ; i++){
        auto r1 =  Menu::nearestNeighborTSP(graph, min, i);
        cout << i <<" : "<< r1.second << " , " << r1.first << endl;
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    cout << duration1.count();
    return 0;
}

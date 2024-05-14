
#include "DisplayMenu.h"
#include "Menu.h"
#include <chrono>

int main() {

    /*DisplayMenu displayMenu = DisplayMenu();
    displayMenu.OpenMenu();*/


    Graph<int>* graph = new Graph<int>();
    vector<int> min;
    Data::loadGraph(graph, "edges2.csv", true);
    cout << "lido " << endl;
    //Data::loadNodesInfo(graph, "nodes.csv");
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 100 ; i++){
        long double r1 =  Menu::nearestNeighborTSP(graph, min, i);
        cout << i <<" : "<< r1 << endl;
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    /*graph = new Graph<int>();
    Data::loadGraph(graph, "edges_25.csv", false);
    Data::loadNodesInfo(graph, "nodes.csv");
    start = std::chrono::high_resolution_clock::now();
    double r2 = Menu::Cristofides(graph, min);
    end = std::chrono::high_resolution_clock::now();
    auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    cout << r1 << " and " << r2 << endl;
    cout << (r2/r1) << endl;
    cout << duration1.count() << " ; " << duration2.count() << endl;*/

    return 0;
}

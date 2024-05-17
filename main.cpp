
#include "DisplayMenu.h"



int main() {
    DisplayMenu displayMenu = DisplayMenu();
    displayMenu.SelectGraphType();

    /*
    double minDist; vector<int> minPath;
    chrono::time_point<chrono::high_resolution_clock> start;
    chrono::time_point<chrono::high_resolution_clock> end;
    std::chrono::duration<double> time;


    Graph<int>* graph = new Graph<int>();
    Data::loadGraph(graph, "edges_200.csv", false);
    Data::loadNodesInfo(graph, "nodes.csv");
    start = std::chrono::high_resolution_clock::now();
    // ALTERAR AQUI
    minDist = Menu::greedyHeuristica(graph, minPath);
    for (int i = 0; i < 50000; i++) minDist = Menu::randomSwap(graph, minPath, minDist);
    //Menu::twoOpt(graph, minPath, minDist);
    //minDist = Menu::simulatedAnnealing(graph, minPath);
    end = std::chrono::high_resolution_clock::now();
    time = end - start;

    cout << " Distance: " << minDist << std::endl;

    cout << " Path: ";
    for (int i: minPath)
        cout << i << " -> ";
    cout << "0" << endl;

    cout << " Duration: " << time.count() << " seconds" << endl << endl << endl;
     */

    return 0;
}


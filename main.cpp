#include <iostream>

#include "Data.h"
#include "Algorithm.h"

int main() {
    Graph<int> graph;
    Data::loadConnected(&graph, "tourism.csv");

    vector<unsigned int> minPath;
    std::cout << "min" << Algorithm::Backtracking(graph, minPath) << std::endl;

    for (auto x: minPath)
        std::cout << x << " ";
    std::cout << endl;

    return 0;
}

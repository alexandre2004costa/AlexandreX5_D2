#include <iostream>

#include "Data.h"

int main() {
    std::cout << "Hello, World!" << std::endl;

    Graph<int> graph;
    Data::loadConnected(&graph, "shipping.csv");
    return 0;
}

#include <iostream>

#include "Data.h"
#include "DisplayMenu.h"
#include "Menu.h"
#include <chrono>

int main() {

    Graph<int> graph;
    Data::loadConnected(&graph, "edges_25.csv");

    /*DisplayMenu displayMenu = DisplayMenu();
    displayMenu.Base();*/

    auto inicio = std::chrono::high_resolution_clock::now();

    Menu menu = Menu();
    cout << menu.greedyHeuristica(&graph) << endl;

    auto fim = std::chrono::high_resolution_clock::now();
    auto duracao = std::chrono::duration_cast<std::chrono::milliseconds>(fim - inicio);
    std::cout << "Tempo de execucao: " << duracao.count() << " milissegundos" << std::endl;
    return 0;
}

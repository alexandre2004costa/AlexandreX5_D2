
#include "DisplayMenu.h"
#include "Menu.h"
#include <chrono>

int main() {
    /*
    Graph<int> graph;
    Data::loadGraph(&graph, "edges_25.csv", false);
     */

    //auto inicio = std::chrono::high_resolution_clock::now();

    /*Menu menu = Menu();
    cout << menu.greedyHeuristica(&graph) << endl;*/
  
    DisplayMenu displayMenu = DisplayMenu();
    displayMenu.OpenMenu();

    /*
    auto fim = std::chrono::high_resolution_clock::now();
    auto duracao = std::chrono::duration_cast<std::chrono::milliseconds>(fim - inicio);
    std::cout << "Tempo de execucao: " << duracao.count() << " milissegundos" << std::endl;
    */

    return 0;
}

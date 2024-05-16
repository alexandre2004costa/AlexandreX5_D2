
#include "DisplayMenu.h"

int DisplayMenu::CloseMenu(){
    return 0;
}


void DisplayMenu::SelectGraphType() {
    cout << "-----------------------------| Menu |-----------------------------" << endl;
    cout << " Select the graphs: " << endl;
    cout << " 1-> Toy-Graphs" << endl;
    cout << " 2-> Fully Connected Graphs" << endl;
    cout << " 3-> Real-World Graphs" << endl;
    cout << " 0-> Close menu" << endl;

    cout << endl;
    std::cout << " Option: ";
    string k; cin >> k;
    while (!(k == "1" || k == "2" || k == "3" || k == "0")) {
        cout << "Invalid input. Option: ";
        cin >> k;
    }
    cout << endl << endl;

    int option = stoi(k);
    switch (option){
        case 1:
            SelectGraphToy();
            break;
        case 2:
            SelectGraphFully();
            break;
        case 3:
            SelectGraphReal();
            break;
        case 0:
            CloseMenu();
            break;
        default:
            break;
    }
}


void DisplayMenu::SelectGraphToy(){
    cout << "-----------------------------| Menu |-----------------------------" << endl;
    cout << " Select the graph: " << endl;
    cout << " 1-> shipping" << endl;
    cout << " 2-> stadiums" << endl;
    cout << " 3-> tourism" << endl;
    cout << " 0-> Close menu" << endl;

    cout << endl;
    std::cout << " Option: ";
    string k; cin >> k;
    while (!(k == "1" || k == "2" || k == "3" || k == "0")) {
        cout << "Invalid input. Option: ";
        cin >> k;
    }
    cout << endl << endl;

    int option = stoi(k);
    graph = new Graph<int>();
    switch (option){
        case 1:
            Data::loadGraph(graph, "shipping.csv", true);
            Base();
            break;
        case 2:
            Data::loadGraph(graph, "stadiums.csv", true);
            Base();
            break;
        case 3:
            Data::loadGraph(graph, "tourism.csv", true);
            Base();
            break;
        case 0:
            CloseMenu();
            break;
        default:
            break;
    }
}


void DisplayMenu::SelectGraphFully() {
    cout << "-----------------------------| Menu |-----------------------------" << endl;
    cout << " Select the graph: " << endl;
    cout << " 1-> edges_25" << endl;
    cout << " 2-> edges_50" << endl;
    cout << " 3-> edges_75" << endl;
    cout << " 4-> edges_100" << endl;
    cout << " 5-> edges_200" << endl;
    cout << " 6-> edges_300" << endl;
    cout << " 7-> edges_400" << endl;
    cout << " 8-> edges_500" << endl;
    cout << " 9-> edges_600" << endl;
    cout << " 10-> edges_700" << endl;
    cout << " 11-> edges_800" << endl;
    cout << " 12-> edges_900" << endl;
    cout << " 0-> Close menu" << endl;

    cout << endl;
    std::cout << " Option: ";
    string k; cin >> k;
    while (!(k == "1" || k == "2" || k == "3" || k == "4" || k == "5" || k == "6" ||
        k == "7" || k == "8" || k == "9" || k == "10" || k == "11" || k == "12" || k == "0")) {
        cout << "Invalid input. Option: ";
        cin >> k;
    }
    cout << endl << endl;

    int option = stoi(k);
    graph = new Graph<int>();
    switch (option){
        case 1:
            Data::loadGraph(graph, "edges_25.csv", false);
            Base();
            break;
        case 2:
            Data::loadGraph(graph, "edges_50.csv", false);
            Base();
            break;
        case 3:
            Data::loadGraph(graph, "edges_75.csv", false);
            Base();
            break;
    /** ----------------------- COMPLETAR ---------------------------- **/
        case 0:
            CloseMenu();
            break;
        default:
            break;
    }
}


void DisplayMenu::SelectGraphReal() {
    cout << "-----------------------------| Menu |-----------------------------" << endl;
    cout << " Select the graph: " << endl;
    cout << " 1-> graph 1" << endl;
    cout << " 2-> graph 2" << endl;
    cout << " 3-> graph 3" << endl;
    cout << " 0-> Close menu" << endl;

    cout << endl;
    std::cout << " Option: ";
    string k; cin >> k;
    while (!(k == "1" || k == "2" || k == "3" || k == "0")) {
        cout << "Invalid input. Option: ";
        cin >> k;
    }
    cout << endl << endl;

    int option = stoi(k);
    graph = new Graph<int>();
    switch (option){
        case 1:
            Data::loadGraph(graph, "edges1.csv", true);
            Data::loadNodesInfo(graph, "nodes1.csv");
            Base();
            break;
        case 2:
            Data::loadGraph(graph, "edges2.csv", true);
            Base();
            break;
        case 3:
            Data::loadGraph(graph, "edges3.csv", true);
            Base();
            break;
        case 0:
            CloseMenu();
            break;
        default:
            break;
    }
}


void DisplayMenu::SelectGraphConnect(string nodeFile) {

}



void DisplayMenu::Base(){
    cout << "-----------------------------| Menu |-----------------------------" << endl;
    cout << " 1-> Select the graph" << endl;
    cout << " 2-> Backtracking Algorithm" << endl;
    cout << " 3-> Triangular Approximation Heuristic" << endl;
    cout << " 4-> Other Heuristics" << endl;
    cout << " 5-> TSP in the Real World" << endl;
    cout << " 0-> Close menu" << endl;

    cout << endl;
    std::cout << " Option: ";
    string k; cin >> k;
    while (!(k == "1" || k == "2" || k == "3" || k == "4" || k == "5" || k == "0")) {
        cout << "Invalid input. Option: ";
        cin >> k;
    }
    cout << endl << endl;

    int option = stoi(k);
    double minDist; vector<int> minPath;
    switch (option){
        case 1:
            SelectGraphType();
            break;
        case 2:
            minDist = Menu::Backtracking(*graph, minPath);
            ShowResults(option, minDist, minPath);
            break;
        case 3:
            minDist = Menu::triangularApproximation(graph, minPath);
            //minDist = Menu::nearestNeighborTSP(graph, minPath, 0);
            ShowResults(option, minDist, minPath);
            break;
        case 4:
            minDist = Menu::greedyHeuristica(graph, minPath);
            //minDist = Menu::simulatedAnnealing(graph, minPath);
            //for (int i = 0; i < 100000; i++) {
            //    minDist = Menu::randomSwap(graph, minPath, minDist);
            //}
            //Menu::twoOpt(graph, minPath, minDist);
            ShowResults(option, minDist, minPath);
            break;
        case 5:
            cout << Menu::Cristofides(graph, minPath) << endl;
            askContinue();
            break;
        case 0:
            CloseMenu();
            break;
        default:
            break;
    }
}


void DisplayMenu::ShowResults(int option, double minDist, vector<int> minPath) {
    switch (option){
        case 2:
            cout << "--------------------| Backtracking Algorithm |--------------------" << endl;
            break;
        case 3:
            cout << "--------------| Triangular Approximation Heuristic |--------------" << endl;
            break;
        case 4:
            cout << "-----------------------| Other Heuristics |-----------------------" << endl;
            break;
        case 5:
            cout << "--------------------| TSP in the Real World |---------------------" << endl;
            break;
        default:
            break;
    }

    cout << " Distance: " << minDist << std::endl;

    cout << " Path: ";
    for (int i: minPath)
        cout << i << " -> ";
    cout << "0" << endl << endl << endl;

    askContinue();
}

void DisplayMenu::askContinue() {
    cout << "------------------------------------------------------------------" << endl;
    cout << " Do you want to do something else?" << endl;
    cout << " 1-> Go to menu" << endl;
    cout << " 0-> Close menu" << endl;

    cout << endl;
    std::cout << " Option: ";
    string k; cin >> k;
    while (!(k == "1" || k == "0")) {
        cout << "Invalid input. Option: ";
        cin >> k;
    }
    cout << endl << endl;

    int option = stoi(k);
    switch (option) {
        case 1:
            Base();
            break;
        case 2:
            CloseMenu();
            break;
    }
}

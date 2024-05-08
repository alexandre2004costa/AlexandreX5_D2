
#include "DisplayMenu.h"

void DisplayMenu::OpenMenu(){
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


int DisplayMenu::CloseMenu(){
    return 0;
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
            OpenMenu();
            break;
        case 2:
            minDist = Menu::Backtracking(*graph, minPath);
            ShowResults(option, minDist, minPath);
            break;
        case 3:
            break;
        case 4:
            minDist = Menu::greedyHeuristica(graph, minPath);
            for (int i = 0; i < 100; i++) {
                minDist = Menu::randomSwap(graph, minPath, minDist);
            }
            ShowResults(option, minDist, minPath);
            break;
        case 5:

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

            break;
        default:
            break;
    }

    cout << " Min distance: " << minDist << std::endl;

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

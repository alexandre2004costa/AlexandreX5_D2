//
// Created by Alexandre on 03/05/2024.
//


#include "DisplayMenu.h"

void DisplayMenu::OpenMenu(){

}
void DisplayMenu::Base(){
    cout << "-----------------------------| Menu |-----------------------------" << endl;
    cout << "------------------------ Select the graph ------------------------" << endl;
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
    switch (option){
        case 1:
            main("shipping.csv");
            break;
        case 2:
            main("stadiums.csv");
            break;
        case 3:
            main("tourism.csv");
            break;
        case 0:
            return;
        default:
            break;
    }
}
void DisplayMenu::main(string graph){
    cout << "-----------------------------| Menu |-----------------------------" << endl;
    cout << " 1-> Select the graph" << endl;
    cout << " 2-> Backtracking Algorithm" << endl;
    cout << " 3-> Triangular Approximation Heuristic" << endl;
    cout << " 3-> Other Heuristics" << endl;
    cout << " 3-> TSP in the Real World" << endl;
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

            break;
        case 2:

            break;
        case 3:

            break;
        case 0:
            return;
        default:
            break;
    }
}
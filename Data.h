
#ifndef PROJETODA1_DATA_H
#define PROJETODA1_DATA_H

#include "DataStructures/Graph.h"

#include <string>
#include <fstream>
#include <sstream>
#include <unordered_map>
using namespace std;

class Data {
public:
    static void loadGraph(Graph<int> *graph, string fileName, bool first_line);
};


#endif //PROJETODA1_DATA_H

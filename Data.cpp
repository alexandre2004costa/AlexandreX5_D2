
#include "Data.h"

/**
 * @brief Loads a graph from a file.
 * This function reads data from the file and puts the vertices and edges in the given graph.
 * @param graph Pointer to the graph.
 * @param fileName Name of the file.
 * @param first_line If true ignore the first line.
 * @details
 */
void Data::loadGraph(Graph<int> *graph, string fileName, bool first_line) {
    ifstream in(fileName);
    vector<string> temp;
    string line;
    if (first_line) getline(in, line);  //ignorar a primeira linha

    while (getline(in, line)) {
        istringstream iss(line);
        string eachWord;
        while (getline(iss, eachWord, ',')) temp.push_back(eachWord);

        int infoOrigin = stoi(temp[0]);
        Vertex<int>* origin = graph->findVertex(infoOrigin);
        if (origin == nullptr) graph->addVertex(infoOrigin);

        int infoDestin = stoi(temp[1]);
        Vertex<int>* destin = graph->findVertex(infoDestin);
        if (destin == nullptr) graph->addVertex(infoDestin);

        double dist = stod(temp[2]);
      
        graph->addEdge(infoOrigin, infoDestin, dist);

        temp.clear();
    }

    /*
    for (auto e: graph->findVertex(0)->getAdj()) {
                auto v1 = e->getPair().first;
                auto v2 = e->getPair().second;
                cout << v1->getInfo() << " : " << v2->getInfo() << " - " << e->getWeight() << endl;
    }

    for (auto v: graph->getVertexSet()) {
        for (auto i: v->getAdj()) {
            cout << i->getOrig()->getInfo() << " : " << i->getDest()->getInfo() << " - " << i->getWeight() << endl;
        }
    }
     */
}
void  Data::loadNodesInfo(Graph<int> *graph, string fileName){
    ifstream in(fileName);
    vector<string> temp;
    string line;
    getline(in, line);  //ignorar a primeira linha

    while (getline(in, line)) {
        istringstream iss(line);
        string eachWord;
        while (getline(iss, eachWord, ',')) temp.push_back(eachWord);

        int infoOrigin = stoi(temp[0]);
        Vertex<int>* origin = graph->findVertex(infoOrigin);
        if (origin == nullptr) return;

        double longitude = stod(temp[1]);
        double latitude = stod(temp[2]);

        origin->setLatitude(latitude);
        origin->setLongitude(longitude);

        temp.clear();
    }
}
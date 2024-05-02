
#include "Data.h"


void Data::loadConnected(Graph<int> *graph, string fileName) {
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
        if (origin == nullptr) graph->addVertex(infoOrigin);

        int infoDestin = stoi(temp[1]);
        Vertex<int>* destin = graph->findVertex(infoDestin);
        if (destin == nullptr) graph->addVertex(infoDestin);

        double dist = stod(temp[2]);
        graph->addEdge(infoOrigin, infoDestin, dist);

        temp.clear();
    }

    /*
    for (auto v: graph->getVertexSet()) {
        for (auto i: v->getAdj()) {
            cout << i->getOrig()->getInfo() << " : " << i->getDest()->getInfo() << " - " << i->getWeight() << endl;
        }
    }
     */
}



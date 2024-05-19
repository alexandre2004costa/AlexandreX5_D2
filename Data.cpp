
#include "Data.h"

/**
 * @brief Loads a graph from a file.
 * This function reads data from the file and puts the vertices and edges in the given graph.
 * @param graph Pointer to the graph.
 * @param fileName Name of the file.
 * @param first_line If true ignore the first line.
 * @details Complexity-> O(n), n is the number of lines of the file.
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
}

/**
 * @brief Loads the nodes of a graph from a file.
 * This function reads the nodes information from a file where each line represents a node with its ID, longitude and latitude.
 * It puts the vertices in the given graph.
 * @param graph Pointer to the graph that will have the node information.
 * @param fileName File with the node information.
 */
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
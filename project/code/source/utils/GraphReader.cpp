#include "GraphReader.h"

#include <string>
#include <sstream>
#include <fstream>

GraphReader::GraphReader(Graph * g, std::string folder) : g(g), folder(folder) {}

void GraphReader::readNodes() {
    std::ifstream nodes("../resources/graphs/" + folder + "/nodes.txt");
    std::string line;

    getline(nodes, line);
    int MAX = stoi(line);

    for(int i = 0; i < MAX; i++) {
        double idn, x, y;
        char garbage;

        getline(nodes, line);
        std::stringstream ss(line);

        ss >> garbage >> idn >> garbage >> x >> garbage >> y >> garbage;

        g->addVertex(idn, (int) x, (int) y);
    }

    nodes.close();
}

void GraphReader::readEdges() {
    std::ifstream edges("../resources/graphs/" + folder + "/edges.txt");

    std::string line;

    getline(edges, line);
    int MAX = stoi(line);

    int WEIGHT_CHANGE_THIS = 1;

    for(int i = 0; i < MAX; i++) {
        int idn1, idn2;
        char garbage;

        getline(edges, line);
        stringstream ss(line);

        ss >> garbage >> idn1 >> garbage >> idn2 >> garbage;

        g->addEdge(i, idn1, idn2);
    }

    edges.close();
}

void GraphReader::loadElements() {
    int centralPoint = rand() % g->getNumVertex();
    int catchPoint;

    do {
        catchPoint = rand() % g->getNumVertex();
    } while(catchPoint == centralPoint);

    g->setCentralVertex(centralPoint);
    g->addCatchPoint(catchPoint);
}

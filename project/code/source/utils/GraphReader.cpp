#include "GraphReader.h"

#include <string>
#include <sstream>
#include <fstream>

GraphReader::GraphReader(GraphViewer * gv, Graph * G, std::string folder) : gv(gv), G(G), folder(folder) {}

void GraphReader::readNodes() {
    std::ifstream nodes("../resources/graphs/" + folder + "/nodes.txt");
    std::string line;

    getline(nodes, line);
    int MAX = stoi(line);

    for(int i = 0; i < MAX; i++) {
        int idn, x, y;
        char garbage;

        getline(nodes, line);
        std::stringstream ss(line);

        ss >> garbage >> idn >> garbage >> x >> garbage >> y >> garbage;

        G->addVertex(idn);
        gv->addNode(idn, x, y);
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

        G->addEdge(idn1, idn2, WEIGHT_CHANGE_THIS);

        gv->addEdge(i, idn1, idn2, EdgeType::UNDIRECTED);
    }

    edges.close();
}

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

void GraphReader::readTags() {
    std::ifstream tags("../resources/graphs/" + folder + "/tag.txt");
    std::string line;

    getline(tags, line);
    int MAX = stoi(line);

    for(int i = 0; i < MAX; i++) {
        getline(tags, line);
        if(line == "Central") {
            getline(tags, line);
            int MAX_TAG = stoi(line);
            for(int j = 0; j < MAX_TAG; j++) {
                getline(tags, line);
                g->setCentralVertex(stoi(line));
            }
        }
        else if(line == "Catch") {
            getline(tags, line);
            int MAX_TAG = stoi(line);
            for(int j = 0; j < MAX_TAG; j++) {
                getline(tags, line);
                g->addCatchPoint(stoi(line));
            }
        }
        else if(line == "Destination") {
            getline(tags, line);
            int MAX_TAG = stoi(line);
            for(int j = 0; j < MAX_TAG; j++) {
                getline(tags, line);
                g->setDestinationVertex(stoi(line));
            }
        }
    }

    tags.close();
}

void GraphReader::loadElements() {
    for(Vertex * current : g->getVertexSet()) {
        // Densidade populacional
        // Average Speed
        // Reachable
        // Draw Edges

        for(Edge * adj : current->getAdj()) {
            // Weight Time
            // Weight Distance
            adj->setWeight(1, 1);
        }
    }
}

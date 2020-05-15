#include <fstream>
#include <string>
#include <stdio.h>
#include <sstream>
#include "graphviewer.h"
#include "source/utils/GraphReader.h"
#include "source/utils/GraphDrawer.h"

void readGraph(Graph * graph, string fileName);

void drawGraph(Graph * graph);

int main() {
    string fileName;
    std::cout << "File Name:" << std::endl;
    std::cin >> fileName;
    std::cin.ignore(1000, '\n');

    Graph graph;
    readGraph(&graph, fileName);
    drawGraph(&graph);

    string algorithm;
    std::cout << "Algorithm: " << std::endl;
    std::cin >> algorithm;
    std::cin.ignore(1000, '\n');

    getchar();

    return 0;
}

void readGraph(Graph * graph, string fileName) {
    GraphReader gReader(graph, fileName);
    gReader.readNodes();
    gReader.readEdges();
}

void drawGraph(Graph * graph) {
    GraphViewer * gView = new GraphViewer(1000, 1000, false);
    gView->createWindow(500,500);
    GraphDrawer draw(gView, graph);

    draw.draw();

    gView->rearrange();
}


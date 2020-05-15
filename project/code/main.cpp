#include <fstream>
#include <string>
#include <stdio.h>
#include <sstream>
#include "graphviewer.h"
#include "source/utils/GraphReader.h"
#include "source/utils/GraphDrawer.h"

void readGraph(Graph * graph, string fileName);

void drawGraph(Graph * graph);

int main(int argc, char *argv[]) {
    if(argc != 2) {
        cout << "Vai a outro lado" << endl;
        return -1;
    }

    string fileName = argv[1];

    Graph graph;
    readGraph(&graph, fileName);

    graph.preProcess();

    drawGraph(&graph);

    /*
    string algorithm;
    std::cout << "Algorithm: " << std::endl;
    std::cin >> algorithm;
    std::cin.ignore(1000, '\n');
    */

    getchar();

    return 0;
}

void readGraph(Graph * graph, string fileName) {
    GraphReader gReader(graph, fileName);
    gReader.readNodes();
    gReader.readEdges();
    gReader.readTags();
    gReader.loadElements();
}

void drawGraph(Graph * graph) {
    GraphViewer * gView = new GraphViewer(1920,1080, false);
    gView->createWindow(1920,1080);
    GraphDrawer draw(gView, graph);

    draw.draw();

    gView->rearrange();
}


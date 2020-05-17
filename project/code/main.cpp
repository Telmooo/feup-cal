#include <fstream>
#include <string>
#include <stdio.h>
#include <sstream>
#include "graphviewer.h"
#include "source/utils/GraphReader.h"
#include "source/utils/GraphDrawer.h"

void readGraph(Graph * graph, string fileName);

void applyAlgorithmGraph(Graph * graph, GraphDrawer draw, string algorithm);

int main(int argc, char *argv[]) {
    if(argc != 2) {
        cout << "usage: [grid map path] [algorithm]" << endl;
        cout << "Algorithms available: " << endl;
        cout << "dijkstra" << endl;
        return -1;
    }

    string fileName = argv[1];
    string algorithm = "dijkstra";

    GraphViewer * gView = new GraphViewer(1920,1080, false);
    gView->createWindow(1920,1080);

    Graph graph;
    // Read map information from files
    readGraph(&graph, fileName);
    // Pre Process map
    graph.preProcess();

    // Draw
    GraphDrawer drawer(gView, &graph);

    drawer.drawGraph();
    gView->rearrange();

    // Algorithm
    applyAlgorithmGraph(&graph, drawer, algorithm);

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

void applyAlgorithmGraph(Graph * graph, GraphDrawer draw, string algorithm) {
    // first iteration - entre dois pontos
    if(algorithm == "dijkstra") {
        int centralVertexID = graph->getCentralVertex()->getId();
        int pickUpID = graph->getPickUpPoint().at(0)->getId(); // Check if there is any pick up point.
                                                                    // Should be only one cuz it is first iteration
        int destinationID = graph->getDestinationVertex()->getId();

        graph->dijkstraShortestPath(centralVertexID);
        draw.drawPath(graph->getPathEdgeTo(pickUpID));

        graph->dijkstraShortestPath(pickUpID);
        draw.drawPath(graph->getPathEdgeTo(destinationID));

        graph->dijkstraShortestPath(destinationID);
        draw.drawPath(graph->getPathEdgeTo(centralVertexID));
    }
    else if(algorithm == "a-star") {

    }
    // second iteration
    // third iteration
}


#include <fstream>
#include <string>
#include <stdio.h>
#include <sstream>
#include "graphviewer.h"
#include "source/utils/GraphReader.h"
#include "source/utils/GraphDrawer.h"
#include "source/Service/Waggon.h"

void readGraph(Graph * graph, string fileName);

void algorithmFirstIter(Graph * graph, GraphViewer * gView, GraphDrawer draw, string algorithm);

int main(int argc, char *argv[]) {
    if(argc != 2) {
        cout << "usage: [grid map path] [algorithm]" << endl;
        cout << "Algorithms available: " << endl;
        cout << "dijkstra" << endl;
        return -1;
    }

    string fileName = argv[1];
    string iteration = "one";
    string algorithm = "a-star";

    // Init Screen
    GraphViewer * gView = new GraphViewer(1920,1080, false);
    gView->createWindow(1920,1080);

    // Init Graph
    Graph graph;
    readGraph(&graph, fileName);

    // PreProcess and Draw
    graph.preProcess();

    GraphDrawer drawer(gView, &graph);
    drawer.drawGraph();
    gView->rearrange();

    // Apply Algorithm
    if(iteration == "one")
        algorithmFirstIter(&graph, gView, drawer, algorithm);

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

void algorithmFirstIter(Graph * graph, GraphViewer * gView, GraphDrawer draw, string algorithm) {
    Waggon unique = Waggon(1);

    if(algorithm == "dijkstra") {
        if(graph->getPickUpPoint().empty()) {
            cout << "Map without pickUp Points" << endl;
            return;
        }
        if (graph->getCentralVertex() == NULL) {
            cout << "Map without Central Vertex" << endl;
            return;
        }
        int centralVertexID = graph->getCentralVertex()->getID();

        if (graph->getDestinationVertex() == NULL) {
            cout << "Map without destination" << endl;
            return;
        }
        int destinationID = graph->getDestinationVertex()->getID();

        for(Vertex * pickUP : graph->getPickUpPoint()) {
            cout << "Press any key to next pick up one thief" << endl;
            getchar();
            cin.ignore(1000, '\n');

            draw.cleanLastWaggonPath();

            int pickUpID = pickUP->getID();
            graph->dijkstraShortestPath(centralVertexID, pickUpID);
            draw.drawPath(graph->getPathVertexTo(pickUpID), "black");

            graph->dijkstraShortestPath(pickUpID, destinationID);
            draw.drawPath(graph->getPathVertexTo(destinationID), "cyan");

            graph->dijkstraShortestPath(destinationID, centralVertexID);
            draw.drawPath(graph->getPathVertexTo(centralVertexID), "magenta");

            gView->rearrange();
        }
    }
    else if(algorithm == "a-star") {
        if(graph->getPickUpPoint().empty()) {
            cout << "Map without pickUp Points" << endl;
            return;
        }
        if (graph->getCentralVertex() == NULL) {
            cout << "Map without Central Vertex" << endl;
            return;
        }
        int centralVertexID = graph->getCentralVertex()->getID();

        if (graph->getDestinationVertex() == NULL) {
            cout << "Map without destination" << endl;
            return;
        }
        int destinationID = graph->getDestinationVertex()->getID();

        for(Vertex * pickUP : graph->getPickUpPoint()) {
            cout << "Press any key to next pick up one thief" << endl;
            getchar();
            cin.ignore(1000, '\n');

            draw.cleanLastWaggonPath();

            int pickUpID = pickUP->getID();
            graph->AStar(centralVertexID, pickUpID);
            draw.drawPath(graph->getPathVertexTo(pickUpID), "black");

            graph->AStar(pickUpID, destinationID);
            draw.drawPath(graph->getPathVertexTo(destinationID), "cyan");

            graph->AStar(destinationID, centralVertexID);
            draw.drawPath(graph->getPathVertexTo(centralVertexID), "magenta");

            gView->rearrange();
        }
    }
    // second iteration
    // third iteration
}


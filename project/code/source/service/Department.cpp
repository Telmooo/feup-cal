#include "Department.h"
#include "../utils/GraphReader.h"

Department::Department() {
    this->gView = new GraphViewer(1920,1080, false);
    this->graph = new Graph();
    this->gDrawer = new GraphDrawer(this->gView, this->graph);
}

Department::~Department() {
    delete gView;
    delete graph;
    delete gDrawer;

    for(Waggon * w : waggons) {
        delete w;
    }
}

void Department::initDepartment(string fileName) {
    // Init Screen
    gView->createWindow(1920,1080);

    // Init Graph
    GraphReader gReader(graph, fileName);
    gReader.readNodes();
    gReader.readEdges();
    gReader.readTags();
    gReader.loadElements();

    // PreProcess and Draw
    graph->preProcess();
    gDrawer->drawGraph();
    gView->rearrange();
}

void Department::addWaggon(int capacity) {
    waggons.push_back(new Waggon(capacity));
}

void Department::firstIteration(string algorithm) {
    if(graph->getPickUpPoint().empty()) {
        cout << "Map without pickUp Points" << endl;
        return;
    }
    if (graph->getCentralVertex() == NULL) {
        cout << "Map without Central Vertex" << endl;
        return;
    }
    if (graph->getDestinationVertex() == NULL) {
        cout << "Map without destination" << endl;
        return;
    }

    int centralVertexID = graph->getCentralVertex()->getID();
    int destinationID = graph->getDestinationVertex()->getID();

    if(algorithm == "dijkstra") {
        for(Vertex * pickUP : graph->getPickUpPoint()) {
            cout << "Press any key to next pick up one thief" << endl;
            getchar();
            cin.ignore(1000, '\n');

            gDrawer->cleanLastWaggonPath();

            int pickUpID = pickUP->getID();
            graph->dijkstraShortestPath(centralVertexID, pickUpID);
            gDrawer->drawPath(graph->getPathVertexTo(pickUpID), "b"
                                                                "lack");
            graph->dijkstraShortestPath(pickUpID, destinationID);
            gDrawer->drawPath(graph->getPathVertexTo(destinationID), "cyan");

            graph->dijkstraShortestPath(destinationID, centralVertexID);
            gDrawer->drawPath(graph->getPathVertexTo(centralVertexID), "magenta");

            gView->rearrange();
        }
    }
    else if(algorithm == "a-star") {
        for (Vertex *pickUP : graph->getPickUpPoint()) {
            cout << "Press any key to next pick up one thief" << endl;
            getchar();
            cin.ignore(1000, '\n');

            gDrawer->cleanLastWaggonPath();

            int pickUpID = pickUP->getID();
            graph->AStar(centralVertexID, pickUpID);
            gDrawer->drawPath(graph->getPathVertexTo(pickUpID), "black");

            graph->AStar(pickUpID, destinationID);
            gDrawer->drawPath(graph->getPathVertexTo(destinationID), "cyan");

            graph->AStar(destinationID, centralVertexID);
            gDrawer->drawPath(graph->getPathVertexTo(centralVertexID), "magenta");

            gView->rearrange();
        }
    }
}

void Department::addPickUp(int position) {
    graph->addPickUpPoint(position);
    gDrawer->drawGraph();
    gView->rearrange();
}

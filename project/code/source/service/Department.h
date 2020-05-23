#ifndef MEATWAGONS_DEPARTMENT_H
#define MEATWAGONS_DEPARTMENT_H

#include <vector>
#include <graphviewer.h>
#include "Waggon.h"
#include "../graph/graph.h"
#include "../utils/GraphDrawer.h"
#include "Request.h"

class Department {
private:
    GraphViewer * gView;
    Graph * graph;
    GraphDrawer * gDrawer;
    std::vector<Waggon*> waggons;
    std::vector<Request> requests;

    Vertex* centralVertex = NULL;

    int maxCapacity;

    void addRequest(Request request);
public:
    Department();
    ~Department();

    void initDepartment(string fileName);

    void addWaggon(int capacity);

    void getEdges(const std::vector<Vertex*> &vertices, std::vector<Edge> &edges);

    double getPathDistance(const std::vector<Edge> &edges);
    double getPathTime(const std::vector<Edge> &edges);
    void getRequestTime(const std::vector<Edge> &edges, Request &request);

    // -- Iterations -- //
    void firstIteration(string algorithm);
    void secondIteration(string algorithm);
    void thirdIteration(string algorithm);
    void fourthIteration(string algorithm);

    // -- Complexity -- //
    void dijkstraTime(int n);
    void astarTime(int n);
    void nearestNeighboorTime(int n);

    void addPickUp(int basicString);

    void addRequests(std::string location);

    void setCentralVertex(int position);

    Vertex * getCentralVertex();

    void preProcessRequests();
    void distributeSingleRequestPerService();
    void distributeMultiRequestPerService();

    struct WaggonComparator {
        bool operator() (const Waggon *w1, const Waggon *w2);
    };
    struct WaggonComparatorMultiRequest {
        bool operator() (const Waggon *w1, const Waggon *w2);
    };
};


#endif //MEATWAGONS_DEPARTMENT_H

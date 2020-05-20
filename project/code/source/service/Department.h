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
    std::vector<Waggon *> waggons;
    std::vector<Request> requests;

    int maxCapacity;

    void addRequest(Request request);
public:
    Department();
    ~Department();

    void initDepartment(string fileName);
    void addWaggon(int capacity);

    void firstIteration(string algorithm);

    void addPickUp(int basicString);

    void addRequests(std::string location);
};


#endif //MEATWAGONS_DEPARTMENT_H

#ifndef MEATWAGONS_DEPARTMENT_H
#define MEATWAGONS_DEPARTMENT_H

#include <vector>
#include <graphviewer.h>
#include "Waggon.h"
#include "../graph/graph.h"
#include "../utils/GraphDrawer.h"

class Department {
private:
    GraphViewer * gView;
    Graph * graph;
    GraphDrawer * gDrawer;
    std::vector<Waggon *> waggons;
public:
    Department();
    ~Department();

    void initDepartment(string fileName);
    void addWaggon(int capacity);

    void firstIteration(string algorithm);

    void addPickUp(int basicString);
};


#endif //MEATWAGONS_DEPARTMENT_H

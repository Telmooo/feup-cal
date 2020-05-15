//
// Created by diogo on 15/05/2020.
//

#ifndef MEATWAGONS_GRAPHDRAWER_H
#define MEATWAGONS_GRAPHDRAWER_H


#include <string>
#include <graphviewer.h>
#include "../graph/Graph.h"

class GraphDrawer {
private:
    Graph * g;
    GraphViewer * gv;
public:
    GraphDrawer(GraphViewer * gv, Graph * g);

    void draw();
};


#endif //MEATWAGONS_GRAPHDRAWER_H

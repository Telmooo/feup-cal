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
    vector<Edge> edgesLastWaggon;
public:
    GraphDrawer(GraphViewer * gv, Graph * g);

    void drawGraph();

    void drawPath(vector<Vertex *> path);

    void cleanLastWaggonPath();
};


#endif //MEATWAGONS_GRAPHDRAWER_H

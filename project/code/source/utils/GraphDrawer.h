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
    vector<Vertex> vertexLastWaggon;
public:
    GraphDrawer(GraphViewer * gv, Graph * g);

    void drawGraph();

    void setInterestPoints(const std::vector<Vertex*> &points);

    void drawPath(const std::vector<Edge> &edges, string color);

    void cleanLastWaggonPath();
};


#endif //MEATWAGONS_GRAPHDRAWER_H

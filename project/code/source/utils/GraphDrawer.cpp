//
// Created by diogo on 15/05/2020.
//

#include "GraphDrawer.h"

GraphDrawer::GraphDrawer(GraphViewer *gv, Graph *g) : gv(gv), g(g) {}

void GraphDrawer::draw() {
    // Draw Vertex
    vector<Vertex *> vertexVec = g->getVertexSet();
    for(int i = 0; i < vertexVec.size(); i++) {
        Vertex *vert = vertexVec.at(i);
        gv->addNode(vert->getId(), vert->getX(), vert->getY());
    }

    // Draw Edges
    for(int i = 0; i < vertexVec.size(); i++) {
        Vertex *vert = vertexVec.at(i);
        vector<Edge> adj = vert->getAdj();

        for(int j = 0; j < adj.size(); j++) {
            Edge currentEdge = adj.at(j);
            gv->addEdge(currentEdge.getId(), vert->getId(), currentEdge.getDest()->getId(), EdgeType::DIRECTED);
        }
    }
}


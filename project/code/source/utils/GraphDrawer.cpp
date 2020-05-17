//
// Created by diogo on 15/05/2020.
//

#include "GraphDrawer.h"

GraphDrawer::GraphDrawer(GraphViewer *gv, Graph *g) : gv(gv), g(g) {}

void GraphDrawer::drawGraph() {
    // Draw Vertex
    double yPercent, xPercent;

    vector<Vertex *> vertexVec = g->getVertexSet();
    for(int i = 0; i < vertexVec.size(); i++) {
        Vertex *vert = vertexVec.at(i);

        yPercent = 1.0 - ((vert->getY()- g->getMinY())/(g->getMaxY() - g->getMinY()) * 0.9 + 0.05);
        xPercent = (vert->getX() - g->getMinX())/(g->getMaxX() - g->getMinX()) * 0.9 + 0.05;

        if (vert->getCentral()) {
            gv->setVertexColor(vert->getId(), "yellow");
        }
        else if(vert->getCatchPoint()) {
            gv->setVertexColor(vert->getId(), "pink");
        }
        else if(vert->getDestination()) {
            gv->setVertexColor(vert->getId(), "blue");
        }
        else if (!vert->isVisited()) {
            gv->setVertexColor(vert->getId(), "red");
        } else {
            gv->setVertexColor(vert->getId(), "green");
        }

        gv->addNode(vert->getId(), (int) (xPercent * 1920), (int) (yPercent * 1080));
    }

    // Draw Edges
    for(int i = 0; i < vertexVec.size(); i++) {
        Vertex *vert = vertexVec.at(i);
        vector<Edge *> adj = vert->getAdj();

        for(int j = 0; j < adj.size(); j++) {
            Edge * currentEdge = adj.at(j);
            if (currentEdge->getOpen()) {
                gv->setEdgeColor(currentEdge->getId(), "green");
            } else {
                gv->setEdgeColor(currentEdge->getId(), "red");
            }

            gv->addEdge(currentEdge->getId(), vert->getId(), currentEdge->getDest()->getId(), EdgeType::DIRECTED);
        }
    }
}

void GraphDrawer::drawPath(vector<Edge> path) {
    for(int i = 0; i < path.size(); i++) {
        for(Edge e : path) {
            gv->setEdgeColor(e.getId(), "black");
        }
    }
}


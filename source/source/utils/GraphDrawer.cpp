#include "GraphDrawer.h"

GraphDrawer::GraphDrawer(GraphViewer *gv, Graph *g) : gv(gv), g(g) {
    (this->gv)->defineVertexSize(15);
    this->delayed = false;
}

void GraphDrawer::setDelayed(bool delayed) {
    this->delayed = delayed;
}

void GraphDrawer::drawGraph() {
    // Draw Vertex
    double yPercent, xPercent;

    vector<Vertex *> vertexVec = g->getVertexSet();
    for(int i = 0; i < vertexVec.size(); i++) {
        Vertex *vert = vertexVec.at(i);

        yPercent = 1.0 - ((vert->getPosition().getY() - g->getMinY())/(g->getMaxY() - g->getMinY()) * 0.9 + 0.05);
        xPercent = (vert->getPosition().getX() - g->getMinX())/(g->getMaxX() - g->getMinX()) * 0.9 + 0.05;

        if (vert->isCentral()) {
            gv->setVertexColor(vert->getID(), "yellow");
        }
        else if (!vert->isReachable()) {
            gv->setVertexColor(vert->getID(), "red");
        } else {
            gv->setVertexColor(vert->getID(), "green");
        }
        gv->setVertexLabel(vert->getID(), std::to_string(vert->getID()));
        gv->addNode(vert->getID(), (int) (xPercent * 1920), (int) (yPercent * 1080));
    }

    // Draw Edges
    for(int i = 0; i < vertexVec.size(); i++) {
        Vertex *vert = vertexVec.at(i);
        vector<Edge *> adj = vert->getAdj();

        for(int j = 0; j < adj.size(); j++) {
            Edge * currentEdge = adj.at(j);
            if (currentEdge->isOpen()) {
                gv->setEdgeColor(currentEdge->getID(), "green");
            } else {
                gv->setEdgeColor(currentEdge->getID(), "red");
            }

            gv->addEdge(currentEdge->getID(), vert->getID(), currentEdge->getDest()->getID(), EdgeType::DIRECTED);
        }
    }
}

void GraphDrawer::setInterestPoints(const std::vector<Vertex*> &points) {
    for (Vertex *vert : points) {
        vertexLastWaggon.push_back(*vert);
        if (vert->isCentral()) {
            gv->setVertexColor(vert->getID(), "yellow");
        } else if (vert->isPickUp()) {
            gv->setVertexColor(vert->getID(), "pink");
        } else if (vert->isDestination()) {
            gv->setVertexColor(vert->getID(), "blue");
        } else {
            gv->setVertexColor(vert->getID(), "green");
        }
    }
}

void GraphDrawer::drawPath(const std::vector<Edge> &edges, string color) {
    for (const Edge &edge : edges) {
        edgesLastWaggon.push_back(edge);
        gv->setEdgeColor(edge.getID(), color);
        gv->rearrange();
        if (this->delayed) Sleep(250); // Slow visualization
    }
}

void GraphDrawer::cleanLastWaggonPath() {
    for(Edge e : edgesLastWaggon) {
        gv->setEdgeColor(e.getID(), "green");
    }
    for(Vertex v : vertexLastWaggon) {
        gv->setVertexColor(v.getID(), "green");
    }
    edgesLastWaggon.clear();
    vertexLastWaggon.clear();
}


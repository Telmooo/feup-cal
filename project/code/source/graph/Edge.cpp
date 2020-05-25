#include "Edge.h"

Edge::Edge(int id, Vertex *d): id(id), destinationVertex(d) {
    open = false;
    weightDistance = INF;
    weightTime = INF;
}

/* -------------------------------------------------------------------------
                                GETTERS
/-------------------------------------------------------------------------*/

int Edge::getID() const {
    return id;
}

Vertex *Edge::getDest() const {
    return destinationVertex;
}

double Edge::getWeightDistance() const {
    return weightDistance;
}

double Edge::getWeightTime() const {
    return weightTime;
}

bool Edge::isOpen() {
    return open;
}

/* -------------------------------------------------------------------------
                                SETTERS
/-------------------------------------------------------------------------*/

void Edge::setDest(Vertex *dest) {
    this->destinationVertex = dest;
}

void Edge::setWeight(double distance, double time) {
    this->weightDistance = distance;
    this->weightTime = time;
}

void Edge::setOpen(bool open) {
    this->open = open;
}

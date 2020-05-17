#include "Edge.h"

Edge::Edge(int id, Vertex *d): id(id), destinationVertex(d) {
    open = false;
}

int Edge::getId() {
    return id;
}

Vertex *Edge::getDest() const {
    return destinationVertex;
}

void Edge::setDest(Vertex *dest) {
    Edge::destinationVertex = dest;
}

void Edge::setWeight(double distance, double time) {
    Edge::weightDistance = distance;
    Edge::weightTime = time;
}


double Edge::getWeightDistance() const {
    return weightDistance;
}

double Edge::getWeightTime() const {
    return weightTime;
}


void Edge::setOpen(bool op) {
    open = op;
}

bool Edge::getOpen() {
    return open;
}

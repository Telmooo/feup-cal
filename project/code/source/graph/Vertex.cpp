//
// Created by diogo on 17/05/2020.
//

#include "Vertex.h"

/*************************** Vertex Functions **************************/

Vertex::Vertex(int in, int x, int y): id(in), x(x), y(y) {

}

/*
 * Auxiliary function to add an outgoing edge to a vertex (this),
 * with a given destination vertex (d) and edge weight (w).
 */
void Vertex::addEdge(Edge * e) {
    adj.push_back(e);
}


bool Vertex::operator<(Vertex & vertex) const {
    return this->dist < vertex.dist;
}

int Vertex::getId() const {
    return this->id;
}

int Vertex::getX() const {
    return this->x;
}

int Vertex::getY() const {
    return this->y;
}

double Vertex::getDist() const {
    return this->dist;
}

void Vertex::setDist(double dist) {
    Vertex::dist = dist;
}

Vertex *Vertex::getPath() const {
    return this->path;
}

bool Vertex::getCentral() {
    return central;
}

void Vertex::setCentral(bool ctr) {
    central = ctr;
}
void Vertex::setInfo(int info) {
    Vertex::id = info;
}

void Vertex::setAdj(const vector<Edge *> &adj) {
    Vertex::adj = adj;
}

void Vertex::setPath(Vertex *path) {
    Vertex::path = path;
}

void Vertex::setQueueIndex(int queueIndex) {
    Vertex::queueIndex = queueIndex;
}

void Vertex::setVisited(bool visited) {
    Vertex::visited = visited;
}

void Vertex::setProcessing(bool processing) {
    Vertex::processing = processing;
}

const vector<Edge *> &Vertex::getAdj() const {
    return adj;
}

int Vertex::getQueueIndex() const {
    return queueIndex;
}

bool Vertex::isVisited() const {
    return visited;
}

bool Vertex::isProcessing() const {
    return processing;
}

void Vertex::setCatchPoint(bool b) {
    pickUpPoint = b;
}

bool Vertex::getCatchPoint() {
    return pickUpPoint;
}

void Vertex::setDestination(bool b){
    destination = b;
}

bool Vertex::getDestination() {
    return destination;
}

Vertex::~Vertex() {
    for (Edge *e : adj) {
        delete e;
    }
}
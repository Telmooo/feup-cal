#include "Vertex.h"

/*************************** Vertex Functions **************************/

Vertex::Vertex(int in, int x, int y, double popDensity, double avgSpeed) :
    id(in), position(x, y), fCost(INF), gCost(INF),
    path(NULL), central(false), pickUpPoint(false), destination(false),
    popDensity(popDensity), avgSpeed(avgSpeed), reachable(false), visited(false),
    TSPvisited(false), processing(false), queueIndex(0) { }

Vertex::~Vertex() {
    for (Edge *e : adj) {
        delete e;
    }
}

/*
 * Auxiliary function to add an outgoing edge to a vertex (this),
 * with a given destination vertex (d) and edge weight (w).
 */
void Vertex::addEdge(Edge * e) {
    adj.push_back(e);
}


bool Vertex::operator<(Vertex & vertex) const {
    return this->fCost < vertex.fCost;
}

/* -------------------------------------------------------------------------
                                GETTERS
/-------------------------------------------------------------------------*/
int Vertex::getID() const { return id; }

const Position& Vertex::getPosition() const { return position; }

const std::vector<Edge*>& Vertex::getAdj() const { return adj; }

double Vertex::getFCost() const { return fCost; }

double Vertex::getGCost() const { return gCost; }

const Vertex* Vertex::getPath() const { return path; }

bool Vertex::isCentral() const { return central; }

bool Vertex::isPickUp() const { return pickUpPoint; }

bool Vertex::isDestination() const { return destination; }

bool Vertex::isReachable() const { return reachable; }

double Vertex::getPopulationDensity() const { return popDensity; }

double Vertex::getAvgSpeed() const { return avgSpeed; }

bool Vertex::isVisited() const { return visited; }

bool Vertex::isTSPVisited() const { return TSPvisited; }

bool Vertex::isProcessing() const { return processing; }

int Vertex::getQueueIndex() const { return queueIndex; }

/* -------------------------------------------------------------------------
                                SETTERS
/-------------------------------------------------------------------------*/
void Vertex::setPosition(Position position) { this->position = position; }

void Vertex::setFCost(double fCost) { this->fCost = fCost; }

void Vertex::setGCost(double gCost) { this->gCost = gCost; }

void Vertex::setPath(Vertex *path) { this->path = path; }

void Vertex::setCentral(bool central) { this->central = central; }

void Vertex::setPickUp(bool pickUp) { this->pickUpPoint = pickUp; }

void Vertex::setDestination(bool destination) { this->destination = destination; }

void Vertex::setReachable(bool reachable) { this->reachable = reachable; }

void Vertex::setPopulationDensity(double popDensity) { this->popDensity = popDensity; }

void Vertex::setAvgSpeed(double avgSpeed) { this->avgSpeed = avgSpeed; }

void Vertex::setVisited(bool visited) { this->visited = visited; }

void Vertex::setTSPVisited(bool visited) { this->TSPvisited = visited; }

void Vertex::setProcessing(bool processing) { this->processing = processing; }
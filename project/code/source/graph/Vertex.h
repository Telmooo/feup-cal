#ifndef MEATWAGONS_VERTEX_H
#define MEATWAGONS_VERTEX_H

#include <vector>

#include "../utils/MutablePriorityQueue.h"
#include "../utils/Position.h"

#include "graph.h"

class Edge;

class Vertex {
    int id;						// Vertex ID

    Position position;

    std::vector<Edge *> adj;    // Outgoing Edges

    double fCost;               // F_COST for path-finding algorithms
    double gCost;               // G_COST for A*

    Vertex *path;               // Previous vertex on minimum path

    bool central;               // Waggons' Central
    bool pickUpPoint;           // Pickup location
    bool destination;           // Deliver location
    bool reachable;             // If vertex is reachable

    double popDensity;          // Population density
    double avgSpeed;            // Average speed around the vertice

    bool visited;		        // auxiliary field
    bool processing;        	// auxiliary field

    int queueIndex;      		// required by MutablePriorityQueue

public:
    Vertex(int in, int x, int y);

    virtual ~Vertex();

    bool operator<(Vertex & vertex) const;

    void addEdge(Edge * e);
    /* -------------------------------------------------------------------------
                                GETTERS
    /-------------------------------------------------------------------------*/
    int getID() const;

    const Position& getPosition() const;

    const std::vector<Edge*>& getAdj() const;

    double getFCost() const;

    double getGCost() const;

    const Vertex* getPath() const;

    bool isCentral() const;

    bool isPickUp() const;

    bool isDestination() const;

    bool isReachable() const;

    double getPopulationDensity() const;

    double getAvgSpeed() const;

    bool isVisited() const;

    bool isProcessing() const;

    int getQueueIndex() const;
    /* -------------------------------------------------------------------------
                                SETTERS
    /-------------------------------------------------------------------------*/
    void setPosition(Position position);

    void setFCost(double fCost);

    void setGCost(double gCost);

    void setPath(Vertex *path);

    void setCentral(bool central);

    void setPickUp(bool pickUp);

    void setDestination(bool destination);

    void setReachable(bool reachable);

    void setPopulationDensity(double popDensity);

    void setAvgSpeed(double avgSpeed);

    void setVisited(bool visited);

    void setProcessing(bool processing);

    // required by MutablePriorityQueue
    friend class MutablePriorityQueue<Vertex>;

    friend class Graph;
};


#endif //MEATWAGONS_VERTEX_H

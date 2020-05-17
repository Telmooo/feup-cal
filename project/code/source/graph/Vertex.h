#ifndef MEATWAGONS_VERTEX_H
#define MEATWAGONS_VERTEX_H

#include <vector>
#include <windef.h>
#include "../utils/MutablePriorityQueue.h"

#include "graph.h"

class Edge;

class Vertex {
    int id;						// Vertex ID

    vector<Edge *> adj;		    // Outgoing Edges

    double dist = 0;
    double gCost = 0;
    Vertex *path = NULL;

    bool central = false;       // Central de Waggons
    bool pickUpPoint = false;    // Local a recolher os prinsioneiros
    bool destination = false;    // Local a entregar os prisioneiros

    int density;                // Densidade populacional no vértice
    int averageSpeed;           // Velocidade média no vértice
    bool reachable;             // Se o  vértice é alcançavel

    int x;                      // x position
    int y;                      // y position

    bool open = false;

    bool visited = false;		// auxiliary field
    bool processing = false;	// auxiliary field

    int queueIndex = 0; 		// required by MutablePriorityQueue

public:
    Vertex(int in, int x, int y);

    virtual ~Vertex();

    int getId() const;

    int getX() const;
    int getY() const;

    double getDist() const;

    Vertex *getPath() const;

    void addEdge(Edge * e);

    bool operator<(Vertex & vertex) const;

    void setDist(double dist);

    double getGCost() const;

    void setGCost(double gCost);

    void setCentral(bool ctr);

    void setInfo(int info);

    void setAdj(const vector<Edge *> &adj);

    void setPath(Vertex *path);

    void setQueueIndex(int queueIndex);

    bool isOpen() const;

    void setOpen(bool open);

    void setVisited(bool visited);

    void setProcessing(bool processing);

    const vector<Edge *> &getAdj() const;

    int getQueueIndex() const;

    bool isVisited() const;

    bool isProcessing() const;

    bool getCentral();

    bool getDestination();

    void setDestination(bool b);

    void setCatchPoint(bool b);

    bool getCatchPoint();

    // // required by MutablePriorityQueue
    friend class MutablePriorityQueue<Vertex>;
    friend class Graph;
};


#endif //MEATWAGONS_VERTEX_H

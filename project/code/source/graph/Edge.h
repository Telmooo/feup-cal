#ifndef MEATWAGONS_EDGE_H
#define MEATWAGONS_EDGE_H

#include "Graph.h"

using namespace std;

class Vertex;

class Edge {
    int id;                         // edge identifier
    Vertex *destinationVertex;      // destination vertex
    double weightDistance;          // edge weight - distance
    double weightTime;              // edge weight - time
    bool open;                      // if the conection is open

public:
    Edge(int id, Vertex *d);

    int getID() const;
    Vertex* getDest() const;
    double getWeightDistance() const;
    double getWeightTime() const;
    bool isOpen();

    void setDest(Vertex *dest);
    void setWeight(double distance, double time);
    void setOpen(bool open);

    friend class Vertex;
    friend class Graph;
};


#endif //MEATWAGONS_EDGE_H

#ifndef MEATWAGONS_EDGE_H
#define MEATWAGONS_EDGE_H

#include "Graph.h"

using namespace std;

class Vertex;

class Edge {
    int id;                         // edge identifier
    Vertex * destinationVertex;     // destination vertex
    double weightDistance;          // edge weight distance
    double weightTime;              // edge weight time
    bool open;                      // if the conection is open

public:
    Edge(int id, Vertex *d);

    int getId();

    void setDest(Vertex *dest);
    Vertex * getDest() const;

    void setOpen(bool op);
    bool getOpen();

    void setWeight(double distance, double time);

    double getWeightDistance() const;
    double getWeightTime() const;

    friend class Vertex;
    friend class Graph;
};


#endif //MEATWAGONS_EDGE_H

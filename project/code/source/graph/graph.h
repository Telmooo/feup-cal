/*
 * Graph.h
 */
#ifndef GRAPH_H_
#define GRAPH_H_

#include <vector>
#include <queue>
#include <list>
#include <limits>
#include <cmath>
#include <afxres.h>
#include <iostream>
#include "../utils/MutablePriorityQueue.h"

using namespace std;

class Vertex;
class Edge;
class Graph;

#define INF std::numeric_limits<double>::infinity()

/************************* Vertex  **************************/
class Vertex {
    int id;						// content of the vertex
    vector<Edge > adj;		// outgoing edges

    double dist = 0;
    Vertex *path = NULL;
    int queueIndex = 0; 		// required by MutablePriorityQueue

    bool visited = false;		// auxiliary field
    bool processing = false;	// auxiliary field

public:
    Vertex(int in);
    int getId() const;
    double getDist() const;
    Vertex *getPath() const;

    void addEdge(Vertex *dest, double w);

    bool operator<(Vertex & vertex) const;

    void setDist(double dist);

    void setInfo(int info);

    void setAdj(const vector<Edge> &adj);

    void setPath(Vertex *path);

    void setQueueIndex(int queueIndex);

    void setVisited(bool visited);

    void setProcessing(bool processing);

    const vector<Edge> &getAdj() const;

    int getQueueIndex() const;

    bool isVisited() const;

    bool isProcessing() const;



    // // required by MutablePriorityQueue
    friend class MutablePriorityQueue<Vertex>;
    friend class Graph;
};


/********************** Edge  ****************************/
class Edge {
    Vertex * dest;      // destination vertex
    double weight;         // edge weight
public:
    Edge(Vertex *d, double w);

    Vertex *getDest() const;

    void setDest(Vertex *dest);

    double getWeight() const;

    void setWeight(double weight);

    friend class Vertex;
    friend class Graph;
};


/*************************** Graph  **************************/

class Graph {
    vector<Vertex *> vertexSet;    // vertex set
    vector<vector<double>> D;      // minimum distance matrix
    vector<vector<Vertex *>> P;  // path matrix
public:
    Vertex *findVertex(const int &in) const;
    bool addVertex(const int &in);
    bool addEdge(const int &sourc, const int &dest, double w);
    int getNumVertex() const;
    vector<Vertex *> getVertexSet() const;

    // Fp05 - single source
    void unweightedShortestPath(const int &s);
    void dijkstraShortestPath(const int &s);
    void bellmanFordShortestPath(const int &s);
    vector<int> getPathTo(const int &dest) const;

    // Fp05 - all pairs
    void floydWarshallShortestPath();
    vector<int> getfloydWarshallPath(const int &origin, const int &dest) const;

    friend class Vertex;
    friend class Edge;
};



#endif /* GRAPH_H_ */
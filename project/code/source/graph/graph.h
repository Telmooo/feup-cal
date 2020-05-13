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


template <class T> class Vertex;
class Edge;
class Graph;

#define INF std::numeric_limits<double>::infinity()

/************************* Vertex  **************************/
template<class T>
class Vertex {
    T info;						// content of the vertex
    vector<Edge > adj;		// outgoing edges

    double dist = 0;
    Vertex<T> *path = NULL;
    int queueIndex = 0; 		// required by MutablePriorityQueue

    bool visited = false;		// auxiliary field
    bool processing = false;	// auxiliary field

public:
    Vertex(T in);
    T getInfo() const;
    double getDist() const;
    Vertex *getPath() const;

    void addEdge(Vertex<T> *dest, double w);

    bool operator<(Vertex<T> & vertex) const;

    void setDist(double dist);

    void setInfo(T info);

    void setAdj(const vector<Edge> &adj);

    void setPath(Vertex<T> *path);

    void setQueueIndex(int queueIndex);

    void setVisited(bool visited);

    void setProcessing(bool processing);

    const vector<Edge> &getAdj() const;

    int getQueueIndex() const;

    bool isVisited() const;

    bool isProcessing() const;



    // // required by MutablePriorityQueue
    friend class MutablePriorityQueue<Vertex<T>>;
    friend class Graph;
};


/********************** Edge  ****************************/
class Edge {
    Vertex<int> * dest;      // destination vertex
    double weight;         // edge weight
public:
    Edge(Vertex<int> *d, double w);

    Vertex<int> *getDest() const;

    void setDest(Vertex<int> *dest);

    double getWeight() const;

    void setWeight(double weight);

    friend class Vertex<int>;
    friend class Graph;
};


/*************************** Graph  **************************/

class Graph {
    vector<Vertex<int> *> vertexSet;    // vertex set
    vector<vector<double>> D;      // minimum distance matrix
    vector<vector<Vertex<int>*>> P;  // path matrix
public:
    Vertex<int> *findVertex(const int &in) const;
    bool addVertex(const int &in);
    bool addEdge(const int &sourc, const int &dest, double w);
    int getNumVertex() const;
    vector<Vertex<int> *> getVertexSet() const;

    // Fp05 - single source
    void unweightedShortestPath(const int &s);
    void dijkstraShortestPath(const int &s);
    void bellmanFordShortestPath(const int &s);
    vector<int> getPathTo(const int &dest) const;

    // Fp05 - all pairs
    void floydWarshallShortestPath();
    vector<int> getfloydWarshallPath(const int &origin, const int &dest) const;

    friend class Vertex<int>;
    friend class Edge;
};



#endif /* GRAPH_H_ */
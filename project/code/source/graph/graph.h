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

template <class T> class Edge;
template <class T> class Vertex;

class Graph;

#define INF std::numeric_limits<double>::infinity()

/************************* Vertex  **************************/
template<class T>
class Vertex {
    T info;						// content of the vertex
    vector<Edge<T> > adj;		// outgoing edges

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

    void setAdj(const vector<Edge<T>> &adj);

    void setPath(Vertex<T> *path);

    void setQueueIndex(int queueIndex);

    void setVisited(bool visited);

    void setProcessing(bool processing);

    const vector<Edge<T>> &getAdj() const;

    int getQueueIndex() const;

    bool isVisited() const;

    bool isProcessing() const;



    // // required by MutablePriorityQueue
    friend class MutablePriorityQueue<Vertex<T>>;
    friend class Graph;
};


template <class T>
Vertex<T>::Vertex(T in): info(in) {}

/*
 * Auxiliary function to add an outgoing edge to a vertex (this),
 * with a given destination vertex (d) and edge weight (w).
 */
template <class T>
void Vertex<T>::addEdge(Vertex<T> *d, double w) {
    adj.push_back(Edge<T>(d, w));
}

template <class T>
bool Vertex<T>::operator<(Vertex<T> & vertex) const {
    return this->dist < vertex.dist;
}

template <class T>
T Vertex<T>::getInfo() const {
    return this->info;
}

template <class T>
double Vertex<T>::getDist() const {
    return this->dist;
}

template<class T>
void Vertex<T>::setDist(double dist) {
    Vertex::dist = dist;
}

template <class T>
Vertex<T> *Vertex<T>::getPath() const {
    return this->path;
}

template<class T>
void Vertex<T>::setInfo(T info) {
    Vertex::info = info;
}

template<class T>
void Vertex<T>::setAdj(const vector<Edge<T>> &adj) {
    Vertex::adj = adj;
}

template<class T>
void Vertex<T>::setPath(Vertex<T> *path) {
    Vertex::path = path;
}

template<class T>
void Vertex<T>::setQueueIndex(int queueIndex) {
    Vertex::queueIndex = queueIndex;
}

template<class T>
void Vertex<T>::setVisited(bool visited) {
    Vertex::visited = visited;
}

template<class T>
void Vertex<T>::setProcessing(bool processing) {
    Vertex::processing = processing;
}

template<class T>
const vector<Edge<T>> &Vertex<T>::getAdj() const {
    return adj;
}

template<class T>
int Vertex<T>::getQueueIndex() const {
    return queueIndex;
}

template<class T>
bool Vertex<T>::isVisited() const {
    return visited;
}

template<class T>
bool Vertex<T>::isProcessing() const {
    return processing;
}

/********************** Edge  ****************************/
template<class T>
class Edge {
    Vertex<T> * dest;      // destination vertex
    double weight;         // edge weight
public:
    Edge(Vertex<T> *d, double w);

    Vertex<T> *getDest() const;

    void setDest(Vertex<T> *dest);

    double getWeight() const;

    void setWeight(double weight);

    friend class Vertex<T>;
    friend class Graph;
};
template<class T>
Edge<T>::Edge(Vertex<T> *d, double w): dest(d), weight(w) {}

template<class T>
Vertex<T> *Edge<T>::getDest() const {
    return dest;
}

template<class T>
void Edge<T>::setDest(Vertex<T> *dest) {
    Edge::dest = dest;
}

template<class T>
double Edge<T>::getWeight() const {
    return weight;
}

template<class T>
void Edge<T>::setWeight(double weight) {
    Edge::weight = weight;
}

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
    friend class Edge<int>;
};



#endif /* GRAPH_H_ */
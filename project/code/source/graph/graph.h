#ifndef GRAPH_H_
#define GRAPH_H_

#include <vector>
#include <queue>
#include <list>
#include <limits>
#include <cmath>
#include <iostream>
#include "../utils/MutablePriorityQueue.h"

#include "Edge.h"
#include "Vertex.h"

#define INF std::numeric_limits<double>::infinity()

using namespace std;

class Vertex;
class Graph;
class Edge;

class Graph {
private:
    vector<Vertex*> vertexSet;    // vertex set
    vector<Edge*> edgeSet;        // vertex set
    vector<vector<double>> D;      // minimum distance matrix
    vector<vector<Vertex *>> P;    // path matrix

    double maxX;
    double minX;
    double maxY;
    double minY;

public:
    Graph();

    virtual ~Graph();

    Vertex *findVertex(const int &in) const;

    bool addVertex(const int &in, int x, int y, double popDensity, double avgSpeed);
    bool addEdge(int edgeId, const int &source, const int &dest);

    vector<Vertex *> getVertexSet() const;
    int getNumVertex() const;

    vector<Edge *> getEdgeSet() const;
    int getNumEdges() const;

    // -- Depth First Search
    vector<int> dfsFromOrigin(int origin);
    void dfsVisit(Vertex *v, vector<int> & res);

    /* PreProcessGraph */
    void preProcess(int central);
    void resetConnections();

    // --- Transpose Graph
    Graph transpose();

    void clearAuxiliary();

    // --- Strongly Connected Components
    void kosarajuSCC(int origin);

    /* Shortest Path ALgorithms */
    // --- Get Path From Dest
    vector<Vertex *> getPathVertexTo(int dest) const;

    // --- Single source
    void unweightedShortestPath(const int &s);
    void dijkstraShortestPath(const int &s);
    void dijkstraShortestPath(int origin, int dest);
    void bellmanFordShortestPath(const int &s);

    // --- A*
    double heuristic(Vertex *v, Vertex *d);
    Vertex* initAstar(int origin);
    void AStar(int from, int to);

    // -- All pairs
    void floydWarshallShortestPath();
    vector<int> getfloydWarshallPath(const int &origin, const int &dest) const;

    // -- Miscellaneous
    double getMaxX();
    double getMinX();
    double getMaxY();
    double getMinY();

    friend class Vertex;
    friend class Edge;
};

#endif /* GRAPH_H_ */
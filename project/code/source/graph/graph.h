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

    vector<Edge> adj;		    // outgoing edges

    double dist = 0;
    Vertex *path = NULL;
    int queueIndex = 0; 		// required by MutablePriorityQueue

    bool visited = false;		// auxiliary field
    bool processing = false;	// auxiliary field

    bool central = false;       // Central de Waggons
    bool catchPoint = false;    // Local a recolher os prinsioneiros
    bool destination = false;    // Local a entregar os prisioneiros

    int density;                // Densidade populacional no vértice
    int averageSpeed;           // Velocidade média no vértice
    bool reachable;             // Seo  vértice é alcançavel

    int x;                      // x position
    int y;                      // y position

public:
    Vertex(int in, int x, int y);
    int getId() const;
    int getX() const;
    int getY() const;

    double getDist() const;

    Vertex *getPath() const;

    void addEdge(int edgeId, Vertex *dest);

    bool operator<(Vertex & vertex) const;

    void setDist(double dist);

    void setCentral(bool ctr);

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

    bool getCentral();

    bool getDestination();

    void setDestination(bool b);

    void setCatchPoint(bool b);

    bool getCatchPoint();

    // // required by MutablePriorityQueue
    friend class MutablePriorityQueue<Vertex>;
    friend class Graph;
};


/********************** Edge  ****************************/
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


/*************************** Graph  **************************/

class Graph {
private:
    vector<Vertex *> vertexSet;    // vertex set
    vector<vector<double>> D;      // minimum distance matrix
    vector<vector<Vertex *>> P;     // path matrix

    Vertex * centralVertex;
    vector<Vertex *> catchPoints;
    Vertex * destinationVertex;

    double maxX;
    double minX;
    double maxY;
    double minY;

public:
    Vertex *findVertex(const int &in) const;

    bool addVertex(const int &in, int x, int y);

    bool addEdge(int edgeId, const int &sourc, const int &dest);

    vector<Vertex *> getVertexSet() const;
    int getNumVertex() const;

    void setCentralVertex(int position);
    void addCatchPoint(int position);
    void setDestinationVertex(int position);

    // Fp05 - single source
    void unweightedShortestPath(const int &s);
    void dijkstraShortestPath(const int &s);
    void bellmanFordShortestPath(const int &s);
    vector<int> getPathTo(const int &dest) const;

    // Fp05 - all pairs
    void floydWarshallShortestPath();
    vector<int> getfloydWarshallPath(const int &origin, const int &dest) const;

    // Other
    double getMaxX();
    double getMinX();
    double getMaxY();
    double getMinY();

    friend class Vertex;
    friend class Edge;
};

#endif /* GRAPH_H_ */
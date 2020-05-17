#include "Graph.h"

#include <unordered_map>
#include <algorithm>

Graph::~Graph() {
    for (Vertex *v : vertexSet) {
        delete v;
    }
    for (Edge *e : edgeSet) {
        delete e;
    }
}

/*
 * Auxiliary function to find a vertex with a given content.
 */
Vertex * Graph::findVertex(const int &in) const {
    for (auto v : vertexSet)
        if (v->id == in)
            return v;
    return NULL;
}

/* -------------------------------------------------------------------------
            Edges and Vertex setters and getters
/-------------------------------------------------------------------------*/

int Graph::getNumEdges() const {
    return edgeSet.size();
}

vector<Edge *> Graph::getEdgeSet() const {
    return edgeSet;
}

int Graph::getNumVertex() const {
    return vertexSet.size();
}

vector<Vertex *> Graph::getVertexSet() const {
    return vertexSet;
}

/* -------------------------------------------------------------------------
            Central, PickUp and Destination setters and getters
/-------------------------------------------------------------------------*/

void Graph::setCentralVertex(int position) {
    centralVertex = vertexSet.at(position);
    vertexSet.at(position)->setCentral(true);
}

void Graph::addPickUpPoint(int position) {
    pickUpPoints.push_back(vertexSet.at(position));
    vertexSet.at(position)->setCatchPoint(true);
}

void Graph::setDestinationVertex(int position) {
    destinationVertex = vertexSet.at(position);
    vertexSet.at(position)->setDestination(true);
}

Vertex * Graph::getCentralVertex() {
    return centralVertex;
}

vector<Vertex *> Graph::getPickUpPoint() {
    return pickUpPoints;
}

Vertex *Graph::getDestinationVertex() {
    return destinationVertex;
}

/*
 *  Adds a vertex with a given content or info (in) to a graph (this).
 *  Returns true if successful, and false if a vertex with that content already exists.
 */
bool Graph::addVertex(const int &in, int x, int y) {
    if ( findVertex(in) != NULL)
        return false;

    Vertex* add = new Vertex(in, x, y);
    if (vertexSet.empty()) {
        this->minY = add->getY();
        this->minX = add->getX();
        this->maxX = add->getX();
        this->maxY = add->getY();
    }
    else {
        if(add->getX() > maxX)
            maxX = add->getX();
        else if(add->getX() < minX)
            minX = add->getX();
        if(add->getY() > maxY)
            maxY = add->getY();
        else if (add->getY() < minY)
            minY = add->getY();
    }
    vertexSet.push_back(new Vertex(in, x, y));
    return true;
}

/*
 * Adds an edge to a graph (this), given the contents of the source and
 * destination vertices and the edge weight (w).
 * Returns true if successful, and false if the source or destination vertex does not exist.
 */
bool Graph::addEdge(int edgeId, const int &sourc, const int &dest) {
    auto v1 = findVertex(sourc);
    auto v2 = findVertex(dest);
    if (v1 == NULL || v2 == NULL)
        return false;
    Edge * e = new Edge(edgeId, v2);
    edgeSet.push_back(e);
    v1->addEdge(e);
    return true;
}

void Graph::resetConnections() {
    for (Vertex *obj : vertexSet) {
        obj->setVisited(false);
        for (Edge *edge : obj->getAdj()) {
            edge->setOpen(false);
        }
    }
}

/*
 * Performs a depth-first search (dfs) in a graph (this).
 * Returns a vector with the contents of the vertices by dfs order.
 * Follows the algorithm described in theoretical classes.
 */
vector<int> Graph::dfsFromOrigin(int origin) {
    vector<int> res;
    resetConnections();
    Vertex *start = findVertex(origin);
    if (start == NULL) return res;

    dfsVisit(start, res);
    return res;
}

/*
 * Auxiliary function that visits a vertex (v) and its adjacent not yet visited, recursively.
 * Updates a parameter with the list of visited node contents.
 */
void Graph::dfsVisit(Vertex *v, vector<int> & res) {
    v->setVisited(true);
    res.push_back(v->getId());
    for (Edge *edge : v->adj) {
        edge->setOpen(true);
        if (!(edge->getDest())->visited)
            dfsVisit(edge->getDest(), res);
    }
}

/* -------------------------------------------------------------------------
                            TRANSPOSE
/-------------------------------------------------------------------------*/

Graph Graph::transpose() {
    Graph transposed;

    unordered_map<Vertex*, Vertex*> vertexMap;

    for (Vertex *v : vertexSet) {
        if (transposed.addVertex(v->getId(), v->getX(), v->getY())) {
            vertexMap.insert(pair<Vertex*, Vertex*> (v, transposed.vertexSet.back()));
        }
    }

    for (Vertex *v : vertexSet) {
        Vertex *transposedV = vertexMap.at(v);
        for (Edge *e : v->adj) {
            Vertex *transposedDest = vertexMap.at(e->getDest());
            Edge * newEdge = new Edge(e->getId(), transposedV);
            transposedDest->addEdge(newEdge);
        }
    }

    return transposed;
}

/* -------------------------------------------------------------------------
                Strongly Connected Components - Kosaraju
/-------------------------------------------------------------------------*/

void Graph::kosarajuSCC(int origin) {
    Graph transposed = transpose();

    vector<int> fromOrigin = dfsFromOrigin(origin);

    vector<int> toOrigin = transposed.dfsFromOrigin(origin);

    vector<int> scc;
    for (int id : fromOrigin) {
        if (find(toOrigin.begin(), toOrigin.end(), id) != toOrigin.end()) {
            scc.push_back(id);
        }
    }

    resetConnections();

    for (int id : scc){
        Vertex *v  = findVertex(id);
        v->setVisited(true);
    }

    for (Vertex *v : vertexSet) {
        if (v->isVisited()) {
            for (Edge *e : v->getAdj()) {
                Vertex *dest = e->getDest();
                if (dest->isVisited()) e->setOpen(true);
            }
        }
    }
}

/* -------------------------------------------------------------------------
                                    Pre-process
/-------------------------------------------------------------------------*/

void Graph::preProcess() {
    if (centralVertex == NULL) return;
    kosarajuSCC(centralVertex->getId());
}

/* -------------------------------------------------------------------------
                Single Source Shortest Path algorithms
/-------------------------------------------------------------------------*/

void Graph::unweightedShortestPath(const int &orig) {
    for (Vertex *v : vertexSet) {
        v->dist = INF;
        v->path = NULL;
    }
    Vertex *start = findVertex(orig);
    start->dist = 0;
    queue<Vertex*> q;
    q.push(start);

    while (!q.empty()) {
        Vertex *v = q.front();
        q.pop();
        for (Edge * edge : v->getAdj()) {
            Vertex *w = edge->destinationVertex;
            if (w->dist == INF) {
                q.push(w);
                w->dist = v->dist + 1;
                w->path = v;
            }
        }
    }
}

void Graph::dijkstraShortestPath(const int &origin) {
    for (Vertex *v : vertexSet) {
        v->dist = INF;
        v->path = NULL;
    }

    Vertex *start = findVertex(origin);
    start->dist = 0;
    MutablePriorityQueue<Vertex> q;
    q.insert(start);

    while (!q.empty()) {
        Vertex *v = q.extractMin();
        for (Edge * edge : v->getAdj()) {
            Vertex *w = edge->destinationVertex;
            if (w->dist > v->dist + edge->weightDistance) {
                w->dist = v->dist + edge->weightDistance;
                w->path = v;
                if (w->getQueueIndex() == 0)
                    q.insert(w);
                else
                    q.decreaseKey(w);
            }
        }
    }
}


void Graph::bellmanFordShortestPath(const int &orig) {
    for (Vertex *v : vertexSet) {
        v->dist = INF;
        v->path = NULL;
    }
    Vertex *start = findVertex(orig);
    start->dist = 0;
    for (int i = 1; i < vertexSet.size(); i++) {
        for (Vertex *v : vertexSet) {
            for (Edge * edge : v->adj) {
                Vertex *w = edge->destinationVertex;
                if (w->dist > v->dist + edge->weightDistance) {
                    w->dist = v->dist + edge->weightDistance;
                    w->path = v;
                }
            }
        }
    }
    for (Vertex *v : vertexSet) {
        for (Edge * edge : v->adj) {
            if (v->dist + edge->weightDistance < (edge->destinationVertex)->dist) {
                cerr << "there are cycles of negative weight\n";
                return;
            }
        }
    }
}


vector<Vertex> Graph::getPathVertexTo(int dest) const {
    vector<Vertex> res;
    Vertex * v = findVertex(dest);
    if (v == nullptr || v->dist == INF)
        return res;
    for ( ; v != nullptr; v = v->path)
        res.push_back(*v);
    reverse(res.begin(), res.end());
    return res;
}

vector<Edge> Graph::getPathEdgeTo(int dest) const {
    vector<Edge> res;
    Vertex * v = findVertex(dest);
    if (v == nullptr || v->dist == INF)
        return res;
    while(v->path != nullptr) {
        for(Edge * e : v->getAdj()) {
            if(e->getDest() == v->path) {
                v = v->path;
                res.push_back(*e);
            }
        }
    }
    reverse(res.begin(), res.end());
    return res;
}



/**************** All Pairs Shortest Path  ***************/
/*
template<class T>
void Graph<T>::floydWarshallShortestPath() {

    const int SIZE = vertexSet.size();

    D.resize(SIZE); P.resize(SIZE);
    for (int i = 0; i < SIZE; i++) {
        D[i].resize(SIZE); P[i].resize(SIZE);
        Vertex<T> *v = vertexSet[i];
        for (int j = 0; j < SIZE; j++) {
            if (i == j) { D[i][j] = 0; P[i][j] = v; continue; }
            D[i][j] = INF; P[i][j] = NULL;
            Vertex<T> *w = vertexSet[j];
            for (Edge<T> edge : v->adj) {
                if (edge.dest == w) {
                    D[i][j] = edge.weight;
                    P[i][j] = edge.dest;
                    break;
                }
            }
        }
    }

    for (int k = 0; k < SIZE; k++) {
        for (int i = 0; i < SIZE; i++) {
            for (int j = 0; j < SIZE; j++) {
                if (D[i][k] + D[k][j] < D[i][j]) {
                    D[i][j] = D[i][k] + D[k][j];
                    P[i][j] = P[i][k];
                }
            }
        }
    }
}

template<class T>
vector<T> Graph<T>::getfloydWarshallPath(const T &orig, const T &dest) const {
    vector<T> res;

    int pos_orig = -1, pos_dest = -1;
    Vertex<T> *origV, *destV;
    for (int i = 0; i < vertexSet.size(); i++) {
        Vertex<T> *v = vertexSet[i];
        if (pos_orig == -1 && v->info == orig) {
            pos_orig = i;
            origV = v;
        }
        if (pos_dest == -1 && v->info == dest) {
            pos_dest = i;
            destV = v;
        }
        if (pos_orig != -1 && pos_dest != -1) break;
    }

    if (pos_orig == -1 || pos_dest == -1) return res;

    if (D[pos_orig][pos_dest] == INF) return res;

    res.push_back(origV->info);
    while (pos_orig != pos_dest) {
        origV = P[pos_orig][pos_dest];
        res.push_back(origV->info);
        for (int i = 0; i < vertexSet.size(); i++) {
            if (vertexSet[i] == origV) {
                pos_orig = i;
                break;
            }
        }
    }

    return res;
}
 */

double Graph::getMaxX() {
    return maxX;
}

double Graph::getMinX() {
    return minX;
}

double Graph::getMaxY() {
    return maxY;
}

double Graph::getMinY() {
    return minY;
}

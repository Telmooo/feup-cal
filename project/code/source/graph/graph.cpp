#include "Graph.h"

#include <unordered_map>
#include <algorithm>

Graph::Graph() : centralVertex(NULL),
                 maxX(0), minX(0), maxY(0), minY(0) { }

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

void Graph::setCentralVertex(int id) {
    Vertex *v = findVertex(id);
    if (v != NULL) {
        centralVertex = v;
        v->setCentral(true);
    }
}

void Graph::addPickUpPoint(int id) {
    Vertex *v = findVertex(id);
    if (v != NULL) {
        pickUpPoints.push_back(v);
        v->setPickUp(true);
    }
}

void Graph::setDestinationVertex(int id) {
    Vertex *v = findVertex(id);
    if (v != NULL) {
        destinationVertex = v;
        v->setDestination(true);
    }
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
    if (findVertex(in) != NULL)
        return false;

    if (vertexSet.empty()) {
        this->minY = y;
        this->minX = x;
        this->maxX = x;
        this->maxY = y;
    }
    else {
        maxX = (x > maxX) ? x : maxX;
        maxY = (y > maxY) ? y : maxY;
        minX = (x < minX) ? x : minX;
        minY = (y < minY) ? y : minY;
    }
    vertexSet.push_back(new Vertex(in, x, y));
    return true;
}

/*
 * Adds an edge to a graph (this), given the contents of the source and
 * destination vertices and the edge weight (w).
 * Returns true if successful, and false if the source or destination vertex does not exist.
 */
bool Graph::addEdge(int edgeId, const int &source, const int &dest) {
    auto v1 = findVertex(source);
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
        obj->setReachable(false);
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
    res.push_back(v->getID());
    for (Edge *edge : v->getAdj()) {
        if (!(edge->getDest())->isVisited())
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
        if (transposed.addVertex(v->getID(), v->getPosition().getX(), v->getPosition().getY())) {
            vertexMap.insert(pair<Vertex*, Vertex*> (v, transposed.vertexSet.back()));
        }
    }

    for (Vertex *v : vertexSet) {
        Vertex *transposedV = vertexMap.at(v);
        for (Edge *e : v->adj) {
            Vertex *transposedDest = vertexMap.at(e->getDest());
            Edge * newEdge = new Edge(e->getID(), transposedV);
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
        v->setReachable(true);
    }

    for (Vertex *v : vertexSet) {
        if (v->isReachable()) {
            for (Edge *e : v->getAdj()) {
                Vertex *dest = e->getDest();
                if (dest->isReachable()) e->setOpen(true);
            }
        }
    }
}

/* -------------------------------------------------------------------------
                                    Pre-process
/-------------------------------------------------------------------------*/

void Graph::preProcess() {
    if (centralVertex == NULL) return;
    kosarajuSCC(centralVertex->getID());
}

/* -------------------------------------------------------------------------
                Single Source Shortest Path algorithms
/-------------------------------------------------------------------------*/
void Graph::clearAuxiliary() {
    for (Vertex *v : vertexSet) {
        v->setVisited(false);
        v->setFCost(INF);
        v->setGCost(INF);
        v->setPath(NULL);
    }
}


void Graph::unweightedShortestPath(const int &orig) {
    clearAuxiliary();

    Vertex *start = findVertex(orig);
    if (start == NULL) return;

    start->setFCost(0);
    queue<Vertex*> q;
    q.push(start);

    while (!q.empty()) {
        Vertex *v = q.front();
        q.pop();
        for (Edge * edge : v->getAdj()) {
            Vertex *w = edge->getDest();
            if (w->getFCost() == INF) {
                q.push(w);
                w->setFCost(v->getFCost() + 1);
                w->setPath(v);
            }
        }
    }
}

void Graph::dijkstraShortestPath(const int &origin) {
    clearAuxiliary();

    Vertex *start = findVertex(origin);
    start->setFCost(0);
    MutablePriorityQueue<Vertex> q;
    q.insert(start);

    while (!q.empty()) {
        Vertex *v = q.extractMin();
        v->setVisited(true);

        for (Edge *edge : v->getAdj()) {
            Vertex *w = edge->getDest();
            double tempCost = v->getFCost() + edge->getWeightDistance();
            if (w->getFCost() > tempCost) {
                w->setFCost(tempCost);
                w->setPath(v);
                if (w->getQueueIndex() == 0)
                    q.insert(w);
                else
                    q.decreaseKey(w);
            }
        }
    }
}



void Graph::dijkstraShortestPath(int origin, int dest) {
    clearAuxiliary();

    Vertex *start = findVertex(origin);
    start->setFCost(0);
    MutablePriorityQueue<Vertex> q;
    q.insert(start);

    while (!q.empty()) {
        Vertex *v = q.extractMin();
        v->setVisited(true);

        if (v->getID() == dest) break;

        for (Edge *edge : v->getAdj()) {
            Vertex *w = edge->getDest();
            double tempCost = v->getFCost() + edge->getWeightDistance();
            if (w->getFCost() > tempCost) {
                w->setFCost(tempCost);
                w->setPath(v);
                if (w->getQueueIndex() == 0)
                    q.insert(w);
                else
                    q.decreaseKey(w);
            }
        }
    }

    while (!q.empty()) {
        q.extractMin();
    }
}


void Graph::bellmanFordShortestPath(const int &orig) {
    clearAuxiliary();

    Vertex *start = findVertex(orig);
    start->setFCost(0);
    for (int i = 1; i < vertexSet.size(); i++) {
        for (Vertex *v : vertexSet) {
            for (Edge * edge : v->getAdj()) {
                Vertex *w = edge->getDest();
                double tempCost = v->getFCost() + edge->getWeightDistance();
                if (w->getFCost() > tempCost) {
                    w->setFCost(tempCost);
                    w->setPath(v);
                }
            }
        }
    }
    for (Vertex *v : vertexSet) {
        for (Edge * edge : v->adj) {
            if (v->getFCost() + edge->getWeightDistance() < (edge->getDest())->getFCost()) {
                cerr << "there are cycles of negative weight\n";
                return;
            }
        }
    }
}


vector<Vertex *> Graph::getPathVertexTo(int dest) const {
    vector<Vertex * > res;
    Vertex * v = findVertex(dest);
    if (v == nullptr || v->getFCost() == INF)
        return res;
    for ( ; v != nullptr; v = v->path)
        res.push_back(v);
    reverse(res.begin(), res.end());
    return res;
}

/* -------------------------------------------------------------------------
                                    A*
/-------------------------------------------------------------------------*/
double Graph::heuristic(Vertex *v, Vertex *d) { // euclidean distance for now
    return v->getPosition().distance(d->getPosition());
}

Vertex* Graph::initAstar(int origin) {
    clearAuxiliary();

    Vertex *start = findVertex(origin);
    start->setFCost(0);
    start->setGCost(0);
    return start;
}

void Graph::AStar(int from, int to) {
    Vertex* start = initAstar(from);
    Vertex* dest = findVertex(to);

    start->setFCost(heuristic(start, dest));

    MutablePriorityQueue<Vertex> q;
    q.insert(start);

    while (!q.empty()) {
        Vertex *v = q.extractMin();
        v->setVisited(true);

        if (v->getID() == dest->getID()) {
            break;
        }


        for (Edge *e : v->getAdj()) {
            Vertex *neighbour = e->getDest();

            if (neighbour->isVisited()) continue;

            double tempCost = v->getGCost() + e->getWeightDistance();

            if (neighbour->getGCost() > tempCost) {
                neighbour->setPath(v);
                neighbour->setGCost(tempCost);
                neighbour->setFCost(neighbour->getGCost() + heuristic(neighbour, dest));
                if (neighbour->getQueueIndex() == 0)
                    q.insert(neighbour);
                else
                    q.decreaseKey(neighbour);
            }
        }
    }
    while (!q.empty()) {
        q.extractMin();
    }
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

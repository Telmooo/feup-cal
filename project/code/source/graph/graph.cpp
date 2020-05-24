#include "Graph.h"

#include <unordered_map>
#include <algorithm>

Graph::Graph() : maxX(0), minX(0), maxY(0), minY(0) { }

Graph::~Graph() {
    for (Vertex *v : vertexSet) {
        delete v;
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

int Graph::getNumVertex() const {
    return vertexSet.size();
}

vector<Vertex *> Graph::getVertexSet() const {
    return vertexSet;
}

/*
 *  Adds a vertex with a given content or info (in) to a graph (this).
 *  Returns true if successful, and false if a vertex with that content already exists.
 */
bool Graph::addVertex(const int &in, int x, int y, double popDensity, double avgSpeed) {
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
    vertexSet.push_back(new Vertex(in, x, y, popDensity, avgSpeed));
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
        if (transposed.addVertex(v->getID(), v->getPosition().getX(), v->getPosition().getY(), v->getPopulationDensity(), v->getAvgSpeed())) {
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

void Graph::preProcess(int central) {
    if (findVertex(central) == NULL) return;
    kosarajuSCC(central);
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



void Graph::dijkstraShortestPath(int origin, int dest, double distP, double timeP) {
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
double Graph::heuristic(Vertex *v, Vertex *d) {
    double distC = v->getPosition().distance(d->getPosition());
    double tempC = distC / v->getAvgSpeed();
    return distC+ tempC; // timeC and distC?
}

Vertex* Graph::initAstar(int origin) {
    clearAuxiliary();

    Vertex *start = findVertex(origin);
    start->setFCost(0);
    start->setGCost(0);
    return start;
}

void Graph::AStar(int from, int to, double distP, double timeP) {
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

/* -------------------------------------------------------------------------
                                    TSP
/-------------------------------------------------------------------------*/
void Graph::clearTSP() {
    for (Vertex *v : vertexSet) {
        v->setTSPVisited(false);
    }
}

void Graph::nearestNeighbour(int from, std::multimap<int, int> &pickUpDestMap, std::vector<Vertex*> &path) {
    clearAuxiliary();
    clearTSP();

    Vertex *start = findVertex(from);
    start->setVisited(true);

    std::set<Vertex*> toVisit;

    for (auto pair : pickUpDestMap) {
        int pickup = pair.first;

        Vertex *current = findVertex(pickup);
        toVisit.insert(current);
    }

    Vertex *current = start;
    while (!toVisit.empty()) {
        Vertex *closest = closestVertex(current, toVisit);

        closest->setTSPVisited(true);

        toVisit.erase(std::find(toVisit.begin(), toVisit.end(), closest));

        if (closest->isPickUp()) {
            std::pair<std::multimap<int, int>::iterator, std::multimap<int, int>::iterator> destinations;
            destinations = pickUpDestMap.equal_range(closest->getID());

            for (std::multimap<int, int>::iterator it = destinations.first; it != destinations.second; it++) {
                int destID = it->second;

                Vertex *dest = findVertex(destID);
                toVisit.insert(dest);
            }

            pickUpDestMap.erase(closest->getID());
        }

        // get path
        std::vector<Vertex*> current_path;

        AStar(current->getID(), closest->getID(), 0, 0); // Change this

        current_path = getPathVertexTo(closest->getID());

        for (Vertex *v : current_path) {
            path.push_back(v);
        }
        //

        current = closest;
    }

    // path back to start
    std::vector<Vertex*> start_path;

    AStar(current->getID(), start->getID(), 0, 0);

    start_path = getPathVertexTo(start->getID());

    for (Vertex *v : start_path) {
        path.push_back(v);
    }
}

Vertex* Graph::closestVertex(Vertex *start, const std::set<Vertex*> &toVisit) {
    if (start == NULL || toVisit.empty()) return NULL;

    auto it = toVisit.begin();
    Vertex *closest = *it;
    double minWeight = heuristic(start, closest);

    it++;

    for (; it != toVisit.end(); it++) {
        Vertex *current = *it;

        double currentWeight = heuristic(start, closest);

        if (currentWeight < minWeight) {
            minWeight = currentWeight;
            closest = current;
        }
    }
    return closest;
}

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

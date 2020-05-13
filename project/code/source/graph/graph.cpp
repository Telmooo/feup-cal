//
// Created by diogo on 13/05/2020.
//
#include "graph.h"

/// VERTEX
Vertex::Vertex(int in): id(in) {}

/*
 * Auxiliary function to add an outgoing edge to a vertex (this),
 * with a given destination vertex (d) and edge weight (w).
 */
void Vertex::addEdge(Vertex *d, double w) {
    adj.push_back(Edge(d, w));
}


bool Vertex::operator<(Vertex & vertex) const {
    return this->dist < vertex.dist;
}

int Vertex::getId() const {
    return this->id;
}

double Vertex::getDist() const {
    return this->dist;
}

void Vertex::setDist(double dist) {
    Vertex::dist = dist;
}

Vertex *Vertex::getPath() const {
    return this->path;
}

void Vertex::setInfo(int info) {
    Vertex::id = info;
}

void Vertex::setAdj(const vector<Edge> &adj) {
    Vertex::adj = adj;
}

void Vertex::setPath(Vertex *path) {
    Vertex::path = path;
}

void Vertex::setQueueIndex(int queueIndex) {
    Vertex::queueIndex = queueIndex;
}

void Vertex::setVisited(bool visited) {
    Vertex::visited = visited;
}

void Vertex::setProcessing(bool processing) {
    Vertex::processing = processing;
}

const vector<Edge> &Vertex::getAdj() const {
    return adj;
}

int Vertex::getQueueIndex() const {
    return queueIndex;
}

bool Vertex::isVisited() const {
    return visited;
}

bool Vertex::isProcessing() const {
    return processing;
}

/// EDGE

Edge::Edge(Vertex *d, double w): dest(d), weight(w) {}

Vertex *Edge::getDest() const {
    return dest;
}

void Edge::setDest(Vertex *dest) {
    Edge::dest = dest;
}

double Edge::getWeight() const {
    return weight;
}

void Edge::setWeight(double weight) {
    Edge::weight = weight;
}

/// GRAPH
int Graph::getNumVertex() const {
    return vertexSet.size();
}

vector<Vertex *> Graph::getVertexSet() const {
    return vertexSet;
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

/*
 *  Adds a vertex with a given content or info (in) to a graph (this).
 *  Returns true if successful, and false if a vertex with that content already exists.
 */
bool Graph::addVertex(const int &in) {
    if ( findVertex(in) != NULL)
        return false;
    vertexSet.push_back(new Vertex(in));
    return true;
}

/*
 * Adds an edge to a graph (this), given the contents of the source and
 * destination vertices and the edge weight (w).
 * Returns true if successful, and false if the source or destination vertex does not exist.
 */
bool Graph::addEdge(const int &sourc, const int &dest, double w) {
    auto v1 = findVertex(sourc);
    auto v2 = findVertex(dest);
    if (v1 == NULL || v2 == NULL)
        return false;
    v1->addEdge(v2,w);
    return true;
}


/**************** Single Source Shortest Path algorithms ************/

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
        for (Edge edge : v->getAdj()) {
            Vertex *w = edge.dest;
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
        for (Edge edge : v->getAdj()) {
            Vertex *w = edge.dest;
            if (w->dist > v->dist + edge.weight) {
                w->dist = v->dist + edge.weight;
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
            for (Edge edge : v->adj) {
                Vertex *w = edge.dest;
                if (w->dist > v->dist + edge.weight) {
                    w->dist = v->dist + edge.weight;
                    w->path = v;
                }
            }
        }
    }
    for (Vertex *v : vertexSet) {
        for (Edge edge : v->adj) {
            if (v->dist + edge.weight < (edge.dest)->dist) {
                cerr << "there are cycles of negative weight\n";
                return;
            }
        }
    }
}


vector<int> Graph::getPathTo(const int &dest) const{
    vector<int> res;
    Vertex *v = findVertex(dest);
    while (v != NULL) {
        res.insert(res.begin(), v->id);
        v = v->path;
    }
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
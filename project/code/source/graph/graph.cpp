//
// Created by diogo on 13/05/2020.
//
#include "graph.h"

int Graph::getNumVertex() const {
    return vertexSet.size();
}

vector<Vertex<int> *> Graph::getVertexSet() const {
    return vertexSet;
}

/*
 * Auxiliary function to find a vertex with a given content.
 */
Vertex<int> * Graph::findVertex(const int &in) const {
    for (auto v : vertexSet)
        if (v->info == in)
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
    vertexSet.push_back(new Vertex<int>(in));
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
    for (Vertex<int> *v : vertexSet) {
        v->dist = INF;
        v->path = NULL;
    }
    Vertex<int> *start = findVertex(orig);
    start->dist = 0;
    queue<Vertex<int>*> q;
    q.push(start);

    while (!q.empty()) {
        Vertex<int> *v = q.front();
        q.pop();
        for (Edge<int> edge : v->getAdj()) {
            Vertex<int> *w = edge.dest;
            if (w->dist == INF) {
                q.push(w);
                w->dist = v->dist + 1;
                w->path = v;
            }
        }
    }
}

void Graph::dijkstraShortestPath(const int &origin) {
    for (Vertex<int> *v : vertexSet) {
        v->dist = INF;
        v->path = NULL;
    }
    Vertex<int> *start = findVertex(origin);
    start->dist = 0;
    MutablePriorityQueue<Vertex<int>> q;
    q.insert(start);

    while (!q.empty()) {
        Vertex<int> *v = q.extractMin();
        for (Edge<int> edge : v->getAdj()) {
            Vertex<int> *w = edge.dest;
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
    for (Vertex<int> *v : vertexSet) {
        v->dist = INF;
        v->path = NULL;
    }
    Vertex<int> *start = findVertex(orig);
    start->dist = 0;
    for (int i = 1; i < vertexSet.size(); i++) {
        for (Vertex<int> *v : vertexSet) {
            for (Edge<int> edge : v->adj) {
                Vertex<int> *w = edge.dest;
                if (w->dist > v->dist + edge.weight) {
                    w->dist = v->dist + edge.weight;
                    w->path = v;
                }
            }
        }
    }
    for (Vertex<int> *v : vertexSet) {
        for (Edge<int> edge : v->adj) {
            if (v->dist + edge.weight < (edge.dest)->dist) {
                cerr << "there are cycles of negative weight\n";
                return;
            }
        }
    }
}


vector<int> Graph::getPathTo(const int &dest) const{
    vector<int> res;
    Vertex<int> *v = findVertex(dest);
    while (v != NULL) {
        res.insert(res.begin(), v->info);
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
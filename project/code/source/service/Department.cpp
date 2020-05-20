#include <fstream>
#include <algorithm>
#include "Department.h"
#include "../utils/GraphReader.h"
#include <functional>

Department::Department() : maxCapacity(0) {

    this->gView = new GraphViewer(1920,1080, false);
    this->graph = new Graph();
    this->gDrawer = new GraphDrawer(this->gView, this->graph);
}

Department::~Department() {
    delete gView;
    delete graph;
    delete gDrawer;

    for (Waggon *w : waggons) {
        delete w;
    }
}

/* -------------------------------------------------------------------------
            Central, PickUp and Destination setters and getters
/-------------------------------------------------------------------------*/

void Department::setCentralVertex(int id) {
    Vertex *v = graph->findVertex(id);
    if (v != NULL) {
        centralVertex = v;
        v->setCentral(true);
    }
}

Vertex * Department::getCentralVertex() {
    return centralVertex;
}

void Department::initDepartment(string fileName) {
    // Init Screen
    gView->createWindow(1920,1080);

    // Init Graph
    GraphReader gReader(graph, fileName);
    gReader.readNodes();
    gReader.readEdges();
    gReader.readTags(*this);
    gReader.loadElements();

    // PreProcess and Draw
    if (centralVertex != NULL)
        graph->preProcess(centralVertex->getID());
    gDrawer->drawGraph();
    gView->rearrange();
}

void Department::addWaggon(int capacity) {
    waggons.push_back(new Waggon(capacity));

    if (capacity > maxCapacity) maxCapacity = capacity;
}

void Department::getEdges(const std::vector<Vertex*> &vertices, std::vector<Edge> &edges) {
    for(int i = 0; i < vertices.size() - 1; i++) {
        for(Edge * e : vertices.at(i)->getAdj()) {
            if(e->getDest()->getID() == vertices.at(i+1)->getID()) {
                edges.push_back(*e);
            }
        }
    }
}

double Department::getPathDistance(const std::vector<Edge> &edges) {
    double distance = 0.0;

    for (const Edge &edge : edges) {
        distance += edge.getWeightDistance();
    }

    return distance;
}

double Department::getPathTime(const std::vector<Edge> &edges) {
    double time = 0;

    for (const Edge &edge : edges) {
        time += edge.getWeightTime();
    }

    return time;
}

void Department::firstIteration(string algorithm) {
    if (getCentralVertex() == NULL) {
        cout << "Map without Central Vertex" << endl;
        return;
    }
    if (waggons.empty()) {
        cout << "No waggons available" << endl;
        return;
    }

    void (Graph::*algFunction)(int, int);

    if (algorithm == "dijkstra") algFunction = &Graph::dijkstraShortestPath;
    else if(algorithm == "a-star") algFunction = &Graph::AStar;
    else {
        cout << "Invalid algorithm" << endl;
        return;
    }

    distributeSingleRequestPerService();

    int centralVertexID = getCentralVertex()->getID();

    for (Waggon *waggon : waggons) {
        int previousEndHour = 0;
        std::vector<Edge> path;
        std::vector<Vertex*> interestPoints;
        for (Service *service : waggon->getServices()) {
            cout << "Press any pre-process the next service" << endl;
            getchar();
            cin.ignore(1000, '\n');
            gDrawer->setInterestPoints(interestPoints);
            gDrawer->cleanLastWaggonPath();

            service->setStartHour(previousEndHour + 1);

            int pickUpID = service->getRequests().at(0).getPickup();
            int destinationID = service->getRequests().at(0).getDest();

            Vertex *pickup = graph->findVertex(pickUpID);
            Vertex *destination = graph->findVertex(destinationID);

            pickup->setPickUp(true);
            destination->setDestination(true);

            interestPoints.clear();
            interestPoints.push_back(pickup); interestPoints.push_back(destination);

            gDrawer->setInterestPoints(interestPoints);
            gView->rearrange();

            cout << "Press any key to process the service" << endl;
            getchar();
            cin.ignore(1000, '\n');

            // ---- PICKUP
            (graph->*algFunction)(centralVertexID, pickUpID);
            getEdges(graph->getPathVertexTo(pickUpID), path);
            service->loadEdges(path);
            service->getRequests().at(0).setPickupHour(getPathTime(path));
            gDrawer->drawPath(path, "black");

            path.clear();
            // ---- DESTINATION
            (graph->*algFunction)(pickUpID, destinationID);
            getEdges(graph->getPathVertexTo(destinationID), path);
            service->loadEdges(path);
            service->getRequests().at(0).setDestHour(service->getRequests().at(0).getPickupHour() + getPathTime(path));
            gDrawer->drawPath(path, "cyan");

            path.clear();
            // ---- RETURN
            (graph->*algFunction)(destinationID, centralVertexID);
            getEdges(graph->getPathVertexTo(centralVertexID), path);
            service->loadEdges(path);
            gDrawer->drawPath(path, "magenta");

            gView->rearrange();

            service->setEndHour(service->getStartHour() + getPathTime(service->getPath()));
            service->setDistance(getPathTime(service->getPath()));

            previousEndHour = service->getEndHour();
            pickup->setPickUp(false);
            destination->setDestination(false);
            path.clear();
        }
    }
}

void Department::secondIteration(string algorithm) {
    firstIteration(algorithm);
}

void Department::thirdIteration(string algorithm, string sub_algorithm) {
}

void Department::addRequest(Request request) {
    if (maxCapacity == 0 || waggons.empty()) {
        cout << "Can't add request. No available Waggons" << endl;
        return;
    }

    while (request.getNumPris() > maxCapacity) {
        Request split = Request(maxCapacity, request.getType(), request.getPickup(),
                                request.getDest(), request.getPDist(), request.getPTime());
        requests.push_back(split);

        request = Request(request.getNumPris() - maxCapacity, request.getType(), request.getPickup(),
                            request.getDest(), request.getPDist(), request.getPTime());
    }
    requests.push_back(request);
}

void Department::addRequests(std::string location) {
    std::ifstream in("../resources/requests/" + location + "/requests.txt");

    if (!in.is_open()) return;

    int N;
    in >> N;
    int numPris, type, pickup, dest;
    double p_dist, p_time;

    for (int i = 0; i < N; i++) {
        in >> numPris >> type >> pickup >> dest >> p_dist >> p_time;

        addRequest(Request(numPris, type, pickup, dest, p_dist, p_time));
    }
}

void Department::preProcessRequests() {
    for (auto it = requests.begin(); it != requests.end(); ) {
        int pickup = it->getPickup();
        int dest = it->getDest();

        Vertex *p = graph->findVertex(pickup);
        Vertex *d = graph->findVertex(dest);

        if (p == NULL || d == NULL || !(p->isReachable() && d->isReachable())) {
            it = requests.erase(it);
        } else {
            it++;
        }

    }

    std::sort(requests.begin(), requests.end(), [](const Request &r1, const Request &r2) { return r1.getNumPris() > r2.getNumPris(); });
}

void Department::distributeSingleRequestPerService() {
    preProcessRequests();

    size_t waggonCounter = 0;
    size_t numWaggons = waggons.size();

    std::priority_queue<Waggon*, std::vector<Waggon*>, WaggonComparator> aux;

    for (Waggon *w : waggons) {
        aux.push(w);
    }

    for (Request request : requests) {

        Waggon *waggon = aux.top();
        aux.pop();

        Service *service = new Service();

        service->addRequest(request);
        service->setEmptySeats(waggon->getCapacity() - request.getNumPris());

        waggon->addService(service);

        waggonCounter++;

        if (waggonCounter == numWaggons) {
            for (Waggon *w : waggons) {
                aux.push(w);
            }
            waggonCounter = 0;
        }
    }
}

bool Department::WaggonComparator::operator() (const Waggon *w1, const Waggon *w2) {
    int w1Left;
    int w2Left;

    if (w1->getServices().empty()) w1Left = w1->getCapacity();
    else w1Left = w1->getServices().back()->getEmptySeats();

    if (w2->getServices().empty()) w2Left = w1->getCapacity();
    else w2Left = w2->getServices().back()->getEmptySeats();

    return w1Left > w2Left;
}

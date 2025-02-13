#include <fstream>
#include <algorithm>
#include <chrono>
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

void Department::setDelayed(bool delayed) {
    gDrawer->setDelayed(delayed);
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
    cout << "Reading...\n";
    gReader.readNodes();
    gReader.readEdges();
    gReader.readTags(*this);
    gReader.loadElements();

    // PreProcess and Draw
    if (centralVertex != NULL) {
        auto start = std::chrono::high_resolution_clock::now();
        cout << "Pre-processing...\n";
        graph->preProcess(centralVertex->getID());
        auto finish = std::chrono::high_resolution_clock::now();
        auto elapsed = chrono::duration_cast<chrono::microseconds>(finish - start).count();
        cout << "Total time pre processing (micro-seconds)=" << elapsed << endl;
    }

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

void Department::getRequestTime(const std::vector<Edge> &edges, Request &request) {
    double time = 0;
    bool pickUpDone = false;

    int pickUp = request.getPickup();
    int dest = request.getDest();

    for (const Edge &edge : edges) {
        time += edge.getWeightTime();
        if (pickUpDone) {
            if ((edge.getDest())->getID() == dest) {
                request.setDestHour(time);
                break;
            }
        } else {
            if ((edge.getDest())->getID() == pickUp) {
                request.setPickupHour(time);
                break;
            }
        }
    }
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

    void (Graph::*algFunction)(int, int, double, double);

    if (algorithm == "dijkstra") algFunction = &Graph::dijkstraShortestPath;
    else if(algorithm == "a-star") algFunction = &Graph::AStar;
    else {
        cout << "Invalid algorithm" << endl;
        return;
    }

    distributeSingleRequestPerService();

    int centralVertexID = getCentralVertex()->getID();

    for (Waggon *waggon : waggons) {
        double previousEndHour = 0;
        std::vector<Edge> path;
        std::vector<Vertex*> interestPoints;
        for (Service *service : waggon->getServices()) {
            cout << "Press any key to pre-process the next service" << endl;
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

            cout << "Start Hour: "<< service->getStartHour() << endl;
            cout << "Number of prisioners: "<< waggon->getCapacity() - service->getEmptySeats() << endl;
            cout << "Press any key to process the service" << endl;
            getchar();
            cin.ignore(1000, '\n');

            // ---- PICKUP
            (graph->*algFunction)(centralVertexID, pickUpID, service->getRequests().at(0).getPDist(), service->getRequests().at(0).getPTime());
            getEdges(graph->getPathVertexTo(pickUpID), path);
            service->loadEdges(path);
            service->getRequests().at(0).setPickupHour(getPathTime(path));
            gDrawer->drawPath(path, "black");

            path.clear();
            // ---- DESTINATION
            (graph->*algFunction)(pickUpID, destinationID, service->getRequests().at(0).getPDist(), service->getRequests().at(0).getPTime());
            getEdges(graph->getPathVertexTo(destinationID), path);
            service->loadEdges(path);
            service->getRequests().at(0).setDestHour(service->getRequests().at(0).getPickupHour() + getPathTime(path));
            gDrawer->drawPath(path, "orange");

            path.clear();
            // ---- RETURN
            (graph->*algFunction)(destinationID, centralVertexID, service->getRequests().at(0).getPDist(), service->getRequests().at(0).getPTime());
            getEdges(graph->getPathVertexTo(centralVertexID), path);
            service->loadEdges(path);
            gDrawer->drawPath(path, "magenta");

            gView->rearrange();

            service->setEndHour(service->getStartHour() + getPathTime(service->getPath()));
            service->setDistance(getPathDistance(service->getPath()));

            previousEndHour = service->getEndHour();
            cout << "End Hour: "<< service->getEndHour() << endl;
            pickup->setPickUp(false);
            destination->setDestination(false);
            path.clear();
        }
    }
}

void Department::secondIteration(string algorithm) {
    if (getCentralVertex() == NULL) {
        cout << "Map without Central Vertex" << endl;
        return;
    }
    if (waggons.empty()) {
        cout << "No waggons available" << endl;
        return;
    }

    void (Graph::*algFunction)(int, int, double, double);

    if (algorithm == "dijkstra") algFunction = &Graph::dijkstraShortestPath;
    else if(algorithm == "a-star") algFunction = &Graph::AStar;
    else {
        cout << "Invalid algorithm" << endl;
        return;
    }

    distributeSingleRequestPerService();

    int centralVertexID = getCentralVertex()->getID();

    size_t serviceIndex = 0;
    bool has_service;

    std::vector<double> previousEndHour(waggons.size());
    std::vector<Vertex *> interestPoints;
    do {
        has_service = false;
        cout << "Press any key to go next wave of services" << endl;
        getchar();
        cin.ignore(1000, '\n');

        gDrawer->setInterestPoints(interestPoints);
        interestPoints.clear();
        gDrawer->cleanLastWaggonPath();
        gView->rearrange();
        for (int i = 0; i < waggons.size(); i++) {
            Waggon *waggon = waggons.at(i);
            cout << "Press any key to pre-process waggon" << endl;
            getchar();
            cin.ignore(1000, '\n');
            if (serviceIndex < waggon->getServices().size()) {
                std::vector<Edge> path;
                has_service = true;
                Service *service = waggon->getServices().at(serviceIndex);

                service->setStartHour(previousEndHour.at(i) + 1);

                int pickUpID = service->getRequests().at(0).getPickup();
                int destinationID = service->getRequests().at(0).getDest();

                Vertex *pickup = graph->findVertex(pickUpID);
                Vertex *destination = graph->findVertex(destinationID);

                pickup->setPickUp(true);
                destination->setDestination(true);

                interestPoints.push_back(pickup);
                interestPoints.push_back(destination);

                gDrawer->setInterestPoints(interestPoints);
                gView->rearrange();

                cout << "Start Hour: "<< service->getStartHour() << endl;
                cout << "Number of prisioners: "<< waggon->getCapacity() - service->getEmptySeats() << endl;
                cout << "Press any key to process the service" << endl;
                getchar();
                cin.ignore(1000, '\n');

                // ---- PICKUP
                (graph->*algFunction)(centralVertexID, pickUpID, service->getRequests().at(0).getPDist(), service->getRequests().at(0).getPTime());
                getEdges(graph->getPathVertexTo(pickUpID), path);
                service->loadEdges(path);
                service->getRequests().at(0).setPickupHour(getPathTime(path));
                gDrawer->drawPath(path, "black");

                path.clear();
                // ---- DESTINATION
                (graph->*algFunction)(pickUpID, destinationID, service->getRequests().at(0).getPDist(), service->getRequests().at(0).getPTime());
                getEdges(graph->getPathVertexTo(destinationID), path);
                service->loadEdges(path);
                service->getRequests().at(0).setDestHour(
                        service->getRequests().at(0).getPickupHour() + getPathTime(path));
                gDrawer->drawPath(path, "orange");

                path.clear();
                // ---- RETURN
                (graph->*algFunction)(destinationID, centralVertexID, service->getRequests().at(0).getPDist(), service->getRequests().at(0).getPTime());
                getEdges(graph->getPathVertexTo(centralVertexID), path);
                service->loadEdges(path);
                gDrawer->drawPath(path, "magenta");

                gView->rearrange();

                service->setEndHour(service->getStartHour() + getPathTime(service->getPath()));
                service->setDistance(getPathDistance(service->getPath()));

                previousEndHour.at(i) = service->getEndHour();
                cout << "End Hour: "<< service->getEndHour() << endl;
                path.clear();
            }
        }

        for (Vertex *v : interestPoints) {
            v->setPickUp(false);
            v->setDestination(false);
        }
        serviceIndex++;
    } while (has_service);
}

void Department::thirdIteration(string algorithm) {
    if (getCentralVertex() == NULL) {
        cout << "Map without Central Vertex" << endl;
        return;
    }
    if (waggons.empty()) {
        cout << "No waggons available" << endl;
        return;
    }

    void (Graph::*algFunction)(int, std::multimap<int, int>&, std::vector<Vertex*>&, double, double);

    if (algorithm == "nearest") algFunction = &Graph::nearestNeighbour;
    else {
        cout << "Invalid algorithm" << endl;
        return;
    }

    distributeMultiRequestPerService();

    int centralVertexID = getCentralVertex()->getID();

    for (Waggon *waggon : waggons) {
        double previousEndHour = 0;
        cout << "Hour: " << previousEndHour << endl;
        std::vector<Edge> path;
        std::vector<Vertex *> interestPoints;
        for (Service *service : waggon->getServices()) {
            cout << "Press any key to pre-process the next service." << endl;
            getchar();
            cin.ignore(1000, '\n');
            gDrawer->setInterestPoints(interestPoints);
            gDrawer->cleanLastWaggonPath();

            service->setStartHour(previousEndHour + 1);
            interestPoints.clear();

            std::multimap<int, int> pickUpDestMap;
            std::multimap<int, int>::iterator it = pickUpDestMap.begin();
            double maxDistP = std::numeric_limits<double>::lowest();
            double maxTimeP = std::numeric_limits<double>::lowest();

            for (Request &request : service->getRequests()) {
                int pickUpID = request.getPickup();
                int destinationID = request.getDest();

                double distP = request.getPDist();
                double timeP = request.getPTime();

                maxDistP = (distP > maxDistP) ? distP : maxDistP;
                maxTimeP = (timeP > maxTimeP) ? timeP : maxTimeP;

                it = pickUpDestMap.insert(it, std::pair<int, int>(pickUpID, destinationID));

                Vertex *pickup = graph->findVertex(pickUpID);
                Vertex *destination = graph->findVertex(destinationID);

                pickup->setPickUp(true);
                destination->setDestination(true);

                interestPoints.push_back(pickup);
                interestPoints.push_back(destination);
            }


            gDrawer->setInterestPoints(interestPoints);
            gView->rearrange();

            cout << "Service Start Hour: " << service->getStartHour() << endl;
            cout << "Number of prisioners: "<< waggon->getCapacity() - service->getEmptySeats() << endl;
            cout << "Press any key to process the service" << endl;
            getchar();
            cin.ignore(1000, '\n');

            std::vector<Vertex*> vertexPath;
            (graph->*algFunction)(centralVertexID, pickUpDestMap, vertexPath, maxDistP, maxTimeP);
            getEdges(vertexPath, path);
            service->loadEdges(path);

            for (Request &request : service->getRequests()) {
                getRequestTime(path, request);
            }

            gDrawer->drawPath(path, "black");

            gView->rearrange();

            service->setEndHour(service->getStartHour() + getPathTime(service->getPath()));
            service->setDistance(getPathDistance(service->getPath()));

            cout << "Service End Hour: " << service->getEndHour() << endl;
            cout << "Service Distance: " << service->getDistance() << endl;

            previousEndHour = service->getEndHour();

            for (Vertex *v : interestPoints) {
                v->setPickUp(false);
                v->setDestination(false);
            }

            path.clear();
        }
    }
}

void Department::fourthIteration(string algorithm) {
    if (getCentralVertex() == NULL) {
        cout << "Map without Central Vertex" << endl;
        return;
    }
    if (waggons.empty()) {
        cout << "No waggons available" << endl;
        return;
    }

    void (Graph::*algFunction)(int, std::multimap<int, int>&, std::vector<Vertex*>&, double, double);

    if (algorithm == "nearest") algFunction = &Graph::nearestNeighbour;
    else {
        cout << "Invalid algorithm" << endl;
        return;
    }

    distributeMultiRequestPerService();

    int centralVertexID = getCentralVertex()->getID();

    size_t serviceIndex = 0;
    bool has_service;

    std::vector<double> previousEndHour(waggons.size());
    std::vector<Vertex *> interestPoints;
    do {
        has_service = false;
        cout << "Press any key to go next wave of services" << endl;
        getchar();
        cin.ignore(1000, '\n');

        gDrawer->setInterestPoints(interestPoints);
        interestPoints.clear();
        gDrawer->cleanLastWaggonPath();
        gView->rearrange();
        for (int i = 0; i < waggons.size(); i++) {
            Waggon *waggon = waggons.at(i);
            cout << "Press any key to pre-process waggon" << endl;
            getchar();
            cin.ignore(1000, '\n');
            if (serviceIndex < waggon->getServices().size()) {
                std::vector<Edge> path;
                has_service = true;
                Service *service = waggon->getServices().at(serviceIndex);

                service->setStartHour(previousEndHour.at(i) + 1);

                int pickUpID = service->getRequests().at(0).getPickup();
                int destinationID = service->getRequests().at(0).getDest();

                std::multimap<int, int> pickUpDestMap;
                std::multimap<int, int>::iterator it = pickUpDestMap.begin();
                double maxDistP = std::numeric_limits<double>::lowest();
                double maxTimeP = std::numeric_limits<double>::lowest();

                for (Request &request : service->getRequests()) {
                    int pickUpID = request.getPickup();
                    int destinationID = request.getDest();

                    double distP = request.getPDist();
                    double timeP = request.getPTime();

                    maxDistP = (distP > maxDistP) ? distP : maxDistP;
                    maxTimeP = (timeP > maxTimeP) ? timeP : maxTimeP;

                    it = pickUpDestMap.insert(it, std::pair<int, int>(pickUpID, destinationID));

                    Vertex *pickup = graph->findVertex(pickUpID);
                    Vertex *destination = graph->findVertex(destinationID);

                    pickup->setPickUp(true);
                    destination->setDestination(true);

                    interestPoints.push_back(pickup);
                    interestPoints.push_back(destination);
                }


                gDrawer->setInterestPoints(interestPoints);
                gView->rearrange();

                gDrawer->setInterestPoints(interestPoints);
                gView->rearrange();

                cout << "Start Hour: "<< service->getStartHour() << endl;
                cout << "Number of prisioners: "<< waggon->getCapacity() - service->getEmptySeats() << endl;
                cout << "Press any key to process the service" << endl;
                getchar();
                cin.ignore(1000, '\n');

                std::vector<Vertex*> vertexPath;
                (graph->*algFunction)(centralVertexID, pickUpDestMap, vertexPath, maxDistP, maxTimeP);
                getEdges(vertexPath, path);
                service->loadEdges(path);

                for (Request &request : service->getRequests()) {
                    getRequestTime(path, request);
                }

                gDrawer->drawPath(path, "black");

                gView->rearrange();

                service->setEndHour(service->getStartHour() + getPathTime(service->getPath()));
                service->setDistance(getPathDistance(service->getPath()));

                cout << "Service End Hour: " << service->getEndHour() << endl;
                cout << "Service Distance: " << service->getDistance() << endl;

                service->getEndHour();
                cout << "End Hour: "<< service->getEndHour() << endl;
                path.clear();
            }
        }

        for (Vertex *v : interestPoints) {
            v->setPickUp(false);
            v->setDestination(false);
        }
        serviceIndex++;
    } while (has_service);
}

double Department::objectiveFunction() {
    double result = 0;
    for (Waggon *waggon : waggons) {
        for (Service *service : waggon->getServices()) {
            double maxDistP = std::numeric_limits<double>::lowest();
            double maxTimeP = std::numeric_limits<double>::lowest();

            for (Request request : service->getRequests()) {
                double distP = request.getPDist();
                double timeP = request.getPTime();
                if (distP > maxDistP) maxDistP = distP;
                if (timeP > maxTimeP) maxTimeP = timeP;
            }

            for (Edge edge : service->getPath()) {
                result += edge.getWeightDistance() * maxDistP + edge.getWeightTime() * maxTimeP;
            }
        }
    }

    return result;
}

void Department::dijkstraTime() {
    int nodes = graph->getNumVertex();

    int dists[] = {10, 307, 818, 1227, 2040, 4080, 6017, 8463, 10198};

    ofstream outFile("../data/dijkstra.csv");

    int origin = 0;
    for (int dist : dists) {

        int dest = (origin + dist) % nodes;

        auto start = std::chrono::high_resolution_clock::now();

        graph->dijkstraShortestPath(origin, dest, 1, 1);

        auto finish = std::chrono::high_resolution_clock::now();
        auto elapsed = chrono::duration_cast<chrono::microseconds>(finish - start).count();
        outFile << dist << "," << elapsed << "\n";
        cout << "Total time (micro-seconds)=" << elapsed << endl;
        cout << "Dijkstra processing grid with " << dist << "\n";
    }
}

void Department::astarTime() {
    int nodes = graph->getNumVertex();

    int dists[] = {10, 307, 818, 1227, 2040, 4080, 6017, 8463, 10198};

    ofstream outFile("../data/astar.csv");
    int origin = 0;

    for (int dist : dists) {

        int dest = (origin + dist) % nodes;

        auto start = std::chrono::high_resolution_clock::now();

        graph->AStar(origin, dest, 1, 1);

        auto finish = std::chrono::high_resolution_clock::now();
        auto elapsed = chrono::duration_cast<chrono::microseconds>(finish - start).count();
        outFile << dist << "," << elapsed << "\n";
        cout << "Total time (micro-seconds)=" << elapsed << endl;
        cout << "A-Star processing grid with " << dist << "\n";
    }
}

void Department::nearestNeighbourTime() {
    int nodes = graph->getNumVertex();

    int sizes[] = {10, 100, 1000, 5000, 10000, 50000, 100000, 200000, 500000, 750000, 1000000};

    ofstream outFile("../data/nearestneighbour.csv");

    for (int size : sizes) {
        vector<Vertex *> garbagePath;
        multimap<int, int> interestPoints;

        for (int i = 1; i < size; i++) {
            int pickup = rand() % nodes;
            int dest = rand() % nodes;
            while (dest == pickup) {
                dest = rand() % nodes;
            }

            interestPoints.insert(std::pair<int, int>(pickup, dest));
        }
        int startNode = rand() % nodes;
        auto start = std::chrono::high_resolution_clock::now();

        graph->nearestNeighbour(startNode, interestPoints, garbagePath, 1, 1);

        auto finish = std::chrono::high_resolution_clock::now();
        auto elapsed = chrono::duration_cast<chrono::microseconds>(finish - start).count();
        outFile << size << "," << elapsed << "\n";
        cout << "Total time (micro-seconds)=" << elapsed << endl;
        cout << "Nearest Neighbour processing grid with " << size << "\n";
    }
}

void Department::kosarajuTime() {
    Graph graph_8x8 = Graph();
    Graph graph_16x16 = Graph();

    GraphReader gReader8x8(&graph_8x8, "8x8");
    gReader8x8.readNodes();
    gReader8x8.readEdges();

    GraphReader gReader16x16(&graph_16x16, "16x16");
    gReader16x16.readNodes();
    gReader16x16.readEdges();

    ofstream outFile("../data/kosaraju.csv");
    Graph* graphs[] = { &graph_8x8, &graph_16x16, this->graph };
    int sizes[] = {8, 16, 100};
    int i = 0;
    for (Graph* graph : graphs) {
        auto start = std::chrono::high_resolution_clock::now();
        graphs[i]->kosarajuSCC(0);
        auto finish = std::chrono::high_resolution_clock::now();
        auto elapsed = chrono::duration_cast<chrono::microseconds>(finish - start).count();
        outFile << sizes[i] << "," << elapsed << "\n";
        cout << "Total time (micro-seconds)=" << elapsed << endl;
        cout << "Kosaraju processing grid " << sizes[i] << "x" << sizes[i] << "\n";
        i++;
    }
}

void Department::distributeRequestTime() {
    int nodes = graph->getNumVertex();
    ofstream outFile("../data/singledistribute.csv");
    ofstream outFile2("../data/multidistribute.csv");

    int sizes[] = {10, 50, 100, 500, 1000, 2000, 3000, 5000};

    for (int size : sizes) {
        requests.clear();
        waggons.clear();

        for (int i = 0; i < 5; i++) {
            this->addWaggon(20);
        }

        for (int i = 0; i < size; i++) {
            Request request = Request(rand() % 20, 1, rand() % nodes, rand() % nodes, 1, 1);
            requests.push_back(request);
        }

        auto start = std::chrono::high_resolution_clock::now();

        this->distributeSingleRequestPerService();

        auto finish = std::chrono::high_resolution_clock::now();
        auto elapsed = chrono::duration_cast<chrono::microseconds>(finish - start).count();
        outFile << size << "," << elapsed << "\n";
        cout << "Total time (micro-seconds)=" << elapsed << endl;
        cout << "Single Request Per Service processing requests " << size << "\n";
    }

    for (int size : sizes) {
        requests.clear();
        waggons.clear();

        for (int i = 0; i < 5; i++) {
            this->addWaggon(20);
        }

        for (int i = 0; i < size; i++) {
            Request request = Request(rand() % 20, 1, rand() % nodes, rand() % nodes, 1, 1);
            requests.push_back(request);
        }

        auto start = std::chrono::high_resolution_clock::now();

        this->distributeSingleRequestPerService();

        auto finish = std::chrono::high_resolution_clock::now();
        auto elapsed = chrono::duration_cast<chrono::microseconds>(finish - start).count();
        outFile2 << size << "," << elapsed << "\n";
        cout << "Total time (micro-seconds)=" << elapsed << endl;
        cout << "Multi Request Per Service processing requests " << size << "\n";
    }
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

    for (auto it = requests.begin(); it != requests.end(); ) {

        Waggon *waggon = aux.top();
        aux.pop();

        Service *service = new Service();

        Request request = *it;

        if (waggon->getCapacity() < request.getNumPris()) {
            Request split = Request(waggon->getCapacity(), request.getType(), request.getPickup(),
                                    request.getDest(), request.getPDist(), request.getPTime());

            service->addRequest(split);
            service->setEmptySeats(waggon->getCapacity() - split.getNumPris());

            request = Request(request.getNumPris() - waggon->getCapacity(), request.getType(), request.getPickup(),
                              request.getDest(), request.getPDist(), request.getPTime());

            it = requests.erase(it);
            it = requests.insert(it, request);
        } else {
            service->addRequest(request);
            service->setEmptySeats(waggon->getCapacity() - request.getNumPris());
            it++;
        }

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

void Department::distributeMultiRequestPerService() {
    preProcessRequests();

    size_t waggonCounter = 0;
    size_t numWaggons = waggons.size();

    std::priority_queue<Waggon*, std::vector<Waggon*>, WaggonComparator> aux;
    std::priority_queue<Waggon*, std::vector<Waggon*>, WaggonComparatorMultiRequest> aux_mult;

    bool has_requests;
    do {
        has_requests = false;

        for (Waggon *w : waggons) {
            aux.push(w);
        }

        for (auto it = requests.begin(); it != requests.end(); ) {
            has_requests = true;

            Waggon *waggon = aux.top();
            aux.pop();

            Service *service = new Service();

            Request request = *it;
            it = requests.erase(it);

            if (waggon->getCapacity() < request.getNumPris()) {
                Request split = Request(waggon->getCapacity(), request.getType(), request.getPickup(),
                                        request.getDest(), request.getPDist(), request.getPTime());

                service->addRequest(split);
                service->setEmptySeats(waggon->getCapacity() - split.getNumPris());

                request = Request(request.getNumPris() - waggon->getCapacity(), request.getType(), request.getPickup(),
                                  request.getDest(), request.getPDist(), request.getPTime());

                it = requests.insert(it, request);
            } else {
                service->addRequest(request);
                service->setEmptySeats(waggon->getCapacity() - request.getNumPris());
            }

            waggon->addService(service);

            waggonCounter++;

            if (waggonCounter == numWaggons) {
                waggonCounter = 0;
                break;
            }
        }

        while (!aux.empty()) {
            aux.pop();
        }

        for (Waggon *waggon : waggons) {
            if (!waggon->getServices().empty() && waggon->getServices().back()->getEmptySeats() > 0) {
                aux_mult.push(waggon);
            }
        }

        while (!aux_mult.empty()) {
            Waggon *waggon = aux_mult.top();
            aux_mult.pop();
            Service *service = waggon->getServices().back();
            for (auto it = requests.begin(); it != requests.end();) {
                has_requests = true;

                Request request = *it;

                if (service->getEmptySeats() >= request.getNumPris()) {
                    service->addRequest(request);
                    service->setEmptySeats(service->getEmptySeats() - request.getNumPris());
                    it = requests.erase(it);
                } else {
                    it++;
                }

                if (service->getEmptySeats() == 0) break;
            }
        }
    }

    while (has_requests);
}

bool Department::WaggonComparator::operator() (const Waggon *w1, const Waggon *w2) {
    return w1->getCapacity() < w2->getCapacity();
}

bool Department::WaggonComparatorMultiRequest::operator() (const Waggon *w1, const Waggon *w2) {
    int w1Left;
    int w2Left;

    if (w1->getServices().empty()) w1Left = w1->getCapacity();
    else {
        w1Left = w1->getServices().back()->getEmptySeats();
        if (w1Left == 0) {
            w1Left = w1->getCapacity();
        }
    }

    if (w2->getServices().empty()) w2Left = w2->getCapacity();
    else {
        w2Left = w2->getServices().back()->getEmptySeats();
        if (w2Left == 0) {
            w2Left = w2->getCapacity();
        }
    }

    return w1Left < w2Left;
}

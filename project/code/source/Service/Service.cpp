#include "Service.h"

Service::Service() {
    emptySeats = 0;
    distance = 0.0;
    startHour = 0;
    endHour = 0;
}

int Service::getEmptySeats() const {
    return emptySeats;
}

void Service::setEmptySeats(int emptySeats) {
    Service::emptySeats = emptySeats;
}

void Service::addRequest(Request &r) {
    requests.push_back(r);
}

std::vector<Request>& Service::getRequests() {
    return requests;
}

void Service::addEdge(const Edge &edge) {
    path.push_back(edge);
}

void Service::loadEdges(const std::vector<Edge> &edges) {
    for (const Edge &edge : edges) {
        addEdge(edge);
    }
}

const std::vector<Edge>& Service::getPath() const {
    return path;
}

double Service::getDistance() const {
    return distance;
}

void Service::setDistance(double distance) {
    Service::distance = distance;
}

double Service::getStartHour() const {
    return startHour;
}

void Service::setStartHour(double startHour) {
    Service::startHour = startHour;
}

double Service::getEndHour() const {
    return endHour;
}

void Service::setEndHour(double endHour) {
    Service::endHour = endHour;
}

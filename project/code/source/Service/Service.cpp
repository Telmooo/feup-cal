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

const std::vector<Request>& Service::getRequests() const {
    return requests;
}

void Service::addEdge(const Edge *edge) {
    path.push_back(edge);
}

const std::vector<const Edge*>& Service::getPath() const {
    return path;
}

double Service::getDistance() const {
    return distance;
}

void Service::setDistance(double distance) {
    Service::distance = distance;
}

int Service::getStartHour() const {
    return startHour;
}

void Service::setStartHour(int startHour) {
    Service::startHour = startHour;
}

int Service::getEndHour() const {
    return endHour;
}

void Service::setEndHour(int endHour) {
    Service::endHour = endHour;
}

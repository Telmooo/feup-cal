#ifndef MEATWAGONS_SERVICE_H
#define MEATWAGONS_SERVICE_H

#include <vector>
#include "Request.h"
#include "../graph/Edge.h"

class Service {
    int emptySeats;
    std::vector<Request> requests;

    std::vector<Edge> path;

    double distance;

    double startHour;
    double endHour;

public:
    Service();

    int getEmptySeats() const;

    void setEmptySeats(int emptySeats);

    void addRequest(Request &r);

    std::vector<Request>& getRequests();

    void addEdge(const Edge &edge);

    void loadEdges(const std::vector<Edge> &edges);

    const std::vector<Edge>& getPath() const;

    double getDistance() const;

    void setDistance(double distance);

    double getStartHour() const;

    void setStartHour(double startHour);

    double getEndHour() const;

    void setEndHour(double endHour);
};

#endif //MEATWAGONS_SERVICE_H

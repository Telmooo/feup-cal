#ifndef MEATWAGONS_SERVICE_H
#define MEATWAGONS_SERVICE_H

#include <vector>
#include "Request.h"
#include "../graph/Edge.h"

class Service {
    int emptySeats;
    std::vector<Request> requests;

    std::vector<const Edge*> path;

    double distance;

    int startHour;
    int endHour;

public:
    Service();

    int getEmptySeats() const;

    void setEmptySeats(int emptySeats);

    void addRequest(Request &r);

    const std::vector<Request>& getRequests() const;

    void addEdge(const Edge *edge);

    const std::vector<const Edge*>& getPath() const;

    double getDistance() const;

    void setDistance(double distance);

    int getStartHour() const;

    void setStartHour(int startHour);

    int getEndHour() const;

    void setEndHour(int endHour);
};

#endif //MEATWAGONS_SERVICE_H

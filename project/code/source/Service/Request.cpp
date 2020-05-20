#include "Request.h"

Request::Request(int numPris, int type, int pickup, int dest, double pDist, double pTime) : numPris(numPris),
                                                                                            type(type), pickup(pickup),
                                                                                            dest(dest), p_dist(pDist),
                                                                                            p_time(pTime) {}

int Request::getNumPris() const {
    return numPris;
}

void Request::setNumPris(int numPris) {
    Request::numPris = numPris;
}

int Request::getType() const {
    return type;
}

void Request::setType(int type) {
    Request::type = type;
}

int Request::getPickup() const {
    return pickup;
}

void Request::setPickup(int pickup) {
    Request::pickup = pickup;
}

int Request::getDest() const {
    return dest;
}

void Request::setDest(int dest) {
    Request::dest = dest;
}

double Request::getPDist() const {
    return p_dist;
}

void Request::setPDist(double pDist) {
    p_dist = pDist;
}

double Request::getPTime() const {
    return p_time;
}

void Request::setPTime(double pTime) {
    p_time = pTime;
}

double Request::getPickupHour() const {
    return pickupHour;
}

void Request::setPickupHour(double pickupHour) {
    Request::pickupHour = pickupHour;
}

double Request::getDestHour() const {
    return destHour;
}

void Request::setDestHour(double destHour) {
    Request::destHour = destHour;
}

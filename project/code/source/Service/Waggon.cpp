#include "Waggon.h"

Waggon::Waggon(int capacity) : capacity(capacity) {}

int Waggon::getCapacity() const {
    return capacity;
}

void Waggon::addService(Service *service) {
    services.push_back(service);
}

std::vector<Service*> Waggon::getServices() const {
    return services;
}

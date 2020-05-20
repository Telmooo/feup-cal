#ifndef MEATWAGONS_WAGGON_H
#define MEATWAGONS_WAGGON_H

#include <vector>
#include "Service.h"

class Waggon {
private:
    int capacity;

    std::vector<Service*> services;
public:
    Waggon(int capacity);

    int getCapacity() const;

    void addService(Service *service);

    std::vector<Service*> getServices() const;
};


#endif //MEATWAGONS_WAGGON_H

#include "source/service/Department.h"

int main(int argc, char *argv[]) {
    if(argc != 2) {
        cout << "usage: [grid map path] [algorithm]" << endl;
        cout << "Algorithms available: " << endl;
        cout << "dijkstra" << endl;
        return -1;
    }

    string location = argv[1];
    string iteration = "three";
    string algorithm = "nearest";

    Department * police = new Department();

    police->initDepartment(location);

    if(iteration == "one") {
        police->addWaggon(5);
        police->addRequests(location);
        police->firstIteration(algorithm);
        getchar();
    }
    else if (iteration == "two") {
        police->addWaggon(5);
        police->addWaggon(8);
        police->addRequests(location);
        police->secondIteration(algorithm);
        getchar();
    }
    else if (iteration == "three") {
        police->addWaggon(10);
        police->addWaggon(8);
        police->addRequests(location);
        police->thirdIteration(algorithm);
        getchar();
    }

    return 0;
}


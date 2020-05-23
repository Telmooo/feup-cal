#include "source/service/Department.h"

int main(int argc, char *argv[]) {
    if(argc != 2) {
        cout << "usage: [grid map path] [iteration] algorithm]" << endl;
        cout << "First and Second iteration algorithms:" << endl;
        cout << "dijkstra" << endl;
        cout << "astar" << endl;
        cout << "Third and Four iteration algorithms:" << endl;
        cout << "nearest" << endl;
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
    else if (iteration == "four") {
        police->addWaggon(10);
        police->addWaggon(8);
        police->addRequests(location);
        police->thirdIteration(algorithm);
        getchar();
    }

    return 0;
}


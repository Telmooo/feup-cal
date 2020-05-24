#include "source/service/Department.h"

int main(int argc, char *argv[]) {
    if(argc != 2) {
        cout << "usage: [grid map path] [iteration] [algorithm]" << endl;
        cout << "First and Second iteration algorithms:" << endl;
        cout << "dijkstra" << endl;
        cout << "a-star" << endl;
        cout << "Third and Four iteration algorithms:" << endl;
        cout << "nearest" << endl;
        return -1;
    }

    string location = "8x8";
    string iteration = "three";
    string algorithm = "nearest";

    Department * police = new Department();

    police->initDepartment(location);

    if (iteration == "one") {
        police->addWaggon(10);
        police->addRequests(location);
        police->firstIteration(algorithm);
        getchar();
    } else if (iteration == "two") {
        police->addWaggon(5);
        police->addWaggon(8);
        police->addRequests(location);
        police->secondIteration(algorithm);
        getchar();
    } else if (iteration == "three") {
        police->addWaggon(10);
        police->addWaggon(8);
        police->addRequests(location);
        police->thirdIteration(algorithm);
        getchar();
    } else if (iteration == "four") {
        /* To do
        police->addWaggon(10);
        police->addWaggon(8);
        police->addRequests(location);
        police->fourthIteration(algorithm);
        getchar();
         */
    }
    else if (iteration == "algorithmTime") {
        if(algorithm == "dijkstra") {
            police->dijkstraTime(4);
            police->dijkstraTime(16);
            police->dijkstraTime(32);
            police->dijkstraTime(64);
            police->dijkstraTime(128);
            /*
            police->dijkstraTime(200);
            police->dijkstraTime(289);
             */
        }
        else if (algorithm == "astar") {
            police->astarTime(4);
            police->astarTime(16);
            police->astarTime(32);
            police->astarTime(64);
            police->astarTime(128);
            police->astarTime(200);
            police->astarTime(289);
        }
        else if (algorithm == "nearest") {
            police->nearestNeighboorTime(4);
            police->nearestNeighboorTime(16);
            police->nearestNeighboorTime(32);
            police->nearestNeighboorTime(64);
            police->nearestNeighboorTime(128);
            police->nearestNeighboorTime(200);
            police->nearestNeighboorTime(289);
        }
    }

    return 0;
}


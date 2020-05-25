#include "source/service/Department.h"

int main(int argc, char *argv[]) {
    if(argc != 5) {
        cout << "usage: [grid map path] [iteration] [algorithm] [delayed]" << endl;
        cout << "First and Second iteration algorithms:" << endl;
        cout << "-   dijkstra" << endl;
        cout << "-   a-star" << endl;
        cout << "Third and Four iteration algorithms:" << endl;
        cout << "-   nearest" << endl;
        return -1;
    }

    string location = "8x8";
    string iteration = "fourth";
    string algorithm = "nearest";

    Department * police = new Department();

    police->initDepartment(location);

    if (std::string(argv[4]) == "true") {
        police->setDelayed(true);
    }

    if (iteration == "first") {
        police->addWaggon(10);
        police->addRequests(location);
        police->firstIteration(algorithm);
        getchar();
    } else if (iteration == "second") {
        police->addWaggon(5);
        police->addWaggon(8);
        police->addRequests(location);
        police->secondIteration(algorithm);
        getchar();
    } else if (iteration == "third") {
        police->addWaggon(10);
        police->addRequests(location);
        police->thirdIteration(algorithm);
        getchar();
    } else if (iteration == "fourth") {
        police->addWaggon(10);
        police->addWaggon(8);
        police->addRequests(location);
        police->fourthIteration(algorithm);
        getchar();
    }
    else if (iteration == "algorithmTime") {
        police->dijkstraTime();
        police->astarTime();
        police->nearestNeighbourTime();
    }

    return 0;
}


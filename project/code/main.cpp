#include "source/service/Department.h"

int main(int argc, char *argv[]) {
    if(argc != 5) {
        cout << "usage: [grid map path] [iteration] [algorithm] [delayed]" << endl;
        cout << "'first' and 'second' iteration algorithms:" << endl;
        cout << "-   dijkstra" << endl;
        cout << "-   a-star" << endl;
        cout << "'third' and 'fourth' iteration algorithms:" << endl;
        cout << "-   nearest" << endl;
        cout << "Example usage:\n";
        cout << "./program_name 8x8 fourth nearest true\n";
        return -1;
    }

    string location = argv[1];
    string iteration = argv[2];
    string algorithm = argv[3];

    if (iteration == "algorithmTime")
        location = "100x100";

    Department * police = new Department();

    police->initDepartment(location);

    if (std::string(argv[4]) == "true") {
        police->setDelayed(true);
    }

    if (iteration == "first") {
        police->addWaggon(10);
        police->addRequests(location);
        police->firstIteration(algorithm);
        cout << "Objective function result: " << police->objectiveFunction() << "\n";
        getchar();
    } else if (iteration == "second") {
        police->addWaggon(5);
        police->addWaggon(8);
        police->addRequests(location);
        police->secondIteration(algorithm);
        cout << "Objective function result: " << police->objectiveFunction() << "\n";
        getchar();
    } else if (iteration == "third") {
        police->addWaggon(10);
        police->addRequests(location);
        police->thirdIteration(algorithm);
        cout << "Objective function result: " << police->objectiveFunction() << "\n";
        getchar();
    } else if (iteration == "fourth") {
        police->addWaggon(10);
        police->addWaggon(8);
        police->addRequests(location);
        police->fourthIteration(algorithm);
        cout << "Objective function result: " << police->objectiveFunction() << "\n";
        getchar();
    }
    else if (iteration == "algorithmTime") {
        police->dijkstraTime();
        police->astarTime();
        police->nearestNeighbourTime();
        police->kosarajuTime();
        police->distributeRequestTime();
    }

    return 0;
}


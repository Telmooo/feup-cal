#include "source/service/Department.h"

int main(int argc, char *argv[]) {
    if(argc != 2) {
        cout << "usage: [grid map path] [algorithm]" << endl;
        cout << "Algorithms available: " << endl;
        cout << "dijkstra" << endl;
        return -1;
    }

    string location = argv[1];
    string iteration = "one";
    string algorithm = "a-star";

    Department * police = new Department();

    police->initDepartment(location);

    if(iteration == "one") {
        police->addWaggon(5);
        police->addRequests(location);
        police->firstIteration(algorithm);
        getchar();
    }
    else if (iteration == "two") {}

    return 0;
}


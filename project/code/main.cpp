#include "source/service/Department.h"

int main(int argc, char *argv[]) {
    if(argc != 2) {
        cout << "usage: [grid map path] [algorithm]" << endl;
        cout << "Algorithms available: " << endl;
        cout << "dijkstra" << endl;
        return -1;
    }

    string fileName = argv[1];
    string iteration = "one";
    string algorithm = "a-star";

    Department * police = new Department();

    police->initDepartment(fileName);

    if(iteration == "one") {
        police->firstIteration(algorithm);
        cout << "Add pickup:" << endl;
        int pos; cin >> pos;
        police->addPickUp(pos);
        police->firstIteration(algorithm);
        getchar();
    }
    else if (iteration == "two") {}


    police->addRequests("8x8");

    return 0;
}


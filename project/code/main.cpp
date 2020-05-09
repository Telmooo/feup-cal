#include <fstream>
#include <string>
#include <stdio.h>
#include <sstream>
#include "graphviewer.h"
#include "source/utils/GraphReader.h"

void drawGraphFromFile(string fileName);

int main() {
    string fileName;
    std::cout << "File Name:" << std::endl;
    std::cin >> fileName;
    std::cin.ignore(1000, '\n');
    drawGraphFromFile(fileName);

    string algorithm;
    std::cout << "Algorithm: " << std::endl;
    std::cin >> algorithm;
    std::cin.ignore(1000, '\n');

    getchar();

    return 0;
}

void drawGraphFromFile(string fileName) {
    GraphViewer * gv = new GraphViewer(1000, 1000, false);
    gv->createWindow(500,500);

    Graph<int> G;
    GraphReader gr(gv, &G, fileName);

    gr.readNodes();
    gr.readEdges();

    gv->rearrange();

    //  Path
    // G.dijkstraShortestPath(20);
}

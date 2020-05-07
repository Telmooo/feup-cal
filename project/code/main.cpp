#include <fstream>
#include <string>
#include <stdio.h>
#include "graphviewer.h"
#include "utils/GraphReader.h"

void drawGraphFromFile();

int main() {
    drawGraphFromFile();

    getchar();

    return 0;
}

void drawGraphFromFile() {
    GraphViewer * gv = new GraphViewer(1200, 1200, false);
    gv->createWindow(1200,1200);

    GraphReader gr(gv, "16x16");

    gr.readNodes();
    gr.readEdges();

    gv->rearrange();
}



#ifndef MEATWAGONS_GRAPHREADER_H
#define MEATWAGONS_GRAPHREADER_H

#include <graphviewer.h>
#include "../graph/graph.h"

class GraphReader {
private:
    Graph<int> * G;
    GraphViewer * gv;
    std::string folder;
public:
    GraphReader(GraphViewer * gv, Graph<int> * G, std::string folder);

    void readNodes();

    void readEdges();
};


#endif //MEATWAGONS_GRAPHREADER_H

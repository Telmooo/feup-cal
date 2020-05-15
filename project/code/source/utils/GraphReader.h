#ifndef MEATWAGONS_GRAPHREADER_H
#define MEATWAGONS_GRAPHREADER_H

#include <graphviewer.h>
#include "../graph/Graph.h"

class GraphReader {
private:
    Graph * g;
    std::string folder;
public:
    GraphReader(Graph * g, std::string folder);

    void readNodes();

    void readEdges();

    void readTags();

    void loadElements();
};


#endif //MEATWAGONS_GRAPHREADER_H

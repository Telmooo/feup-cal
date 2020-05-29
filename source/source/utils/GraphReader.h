#ifndef MEATWAGONS_GRAPHREADER_H
#define MEATWAGONS_GRAPHREADER_H

#include <graphviewer.h>
#include "../graph/Graph.h"
#include "../Service/Department.h"

class GraphReader {
private:
    Graph * g;
    std::string folder;
public:
    GraphReader(Graph * g, std::string folder);

    void readNodes();

    void readEdges();

    void readTags(Department &department);

    void loadElements();
};


#endif //MEATWAGONS_GRAPHREADER_H

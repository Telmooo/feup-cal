#ifndef MEATWAGONS_GRAPHREADER_H
#define MEATWAGONS_GRAPHREADER_H

#include <graphviewer.h>

class GraphReader {
private:
    GraphViewer * gv;
    std::string folder;
public:
    GraphReader(GraphViewer * gv, std::string folder);

    void readNodes();

    void readEdges();
};


#endif //MEATWAGONS_GRAPHREADER_H

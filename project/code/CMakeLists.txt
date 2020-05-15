cmake_minimum_required(VERSION 3.14)
project(meatwagons)

set(CMAKE_CXX_STANDARD 17)

# The line below is necessary if you are under Windows only
# Comment the line below if you are under Linux or Mac OS
link_libraries(ws2_32 wsock32)

include_directories(lib)
include_directories(resources)
add_executable(meatwagons main.cpp lib/connection.cpp lib/graphviewer.cpp source/utils/GraphReader.cpp source/utils/GraphReader.h source/utils/MutablePriorityQueue.h source/graph/Graph.h source/Service/Waggon.cpp source/Service/Waggon.h source/graph/Graph.cpp source/utils/GraphDrawer.cpp source/utils/GraphDrawer.h)
#ifndef ROUTEGRAPH_H
#define ROUTEGRAPH_H
#include <unordered_set>

#include "DataStructures.h"

class RouteGraph {
public:
	RouteGraph(std::shared_ptr<RoadPoint> startingPoint);
	std::shared_ptr<RoadPoint> startingPoint;
	std::unordered_set<std::shared_ptr<RoadPoint>> reachablePoints;
	std::unordered_set<std::shared_ptr<RoadPoint>> endPoints;
	std::shared_ptr<RoadPoint> farthestEndpoint;
	double farthestDistance = 0;
};

#endif /* ROUTEGRAPH_H */


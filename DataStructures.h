#ifndef DATASTRUCTURES_H
#define DATASTRUCTURES_H

#include <vector>
#include <memory>

#include "Constants.h"


struct Connection;

struct RoadPoint
{
	double x, y, z;
	std::vector<std::shared_ptr<Connection>> connections;
};

struct Connection {
	std::shared_ptr<RoadPoint> connectedPoint;
	double horizontalDistance;
	double heightDifference;
	double sinSlope;
	double cosSlope;

	bool hasBikeLane;

	double speeds[MAXIMUM_SPEED_KMH - MINIMUM_SPEED_KMH + 1];
};

struct Route {
	std::shared_ptr<RoadPoint> startingPoint;
	std::vector<std::shared_ptr<Connection>> connections;
	double distance;
};

struct SearchPoint {
	std::shared_ptr<RoadPoint> startingPoint;
	std::shared_ptr<RoadPoint> currentPoint;
	std::vector<std::shared_ptr<Connection>> connections;
	double distance;
	double speed;
	double energy;
};

#endif // DATASTRUCTURES_H

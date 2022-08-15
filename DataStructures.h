#ifndef DATASTRUCTURES_H
#define DATASTRUCTURES_H

#include <vector>

#include "Constants.h"


struct Connection;

struct RoadPoint
{
	double x, y, z;
	std::vector<Connection*> connections;

	~RoadPoint() {
		for (Connection* connection : connections) {
			delete connection;
		}
	}
};

struct Connection {
	RoadPoint* connectedPoint;
	double horizontalDistance;
	double heightDifference;
	double sinSlope;
	double cosSlope;

	double speeds[MAXIMUM_SPEED_KMH - MINIMUM_SPEED_KMH + 1];
};

struct Route {
	RoadPoint* startingPoint;
	std::vector<Connection*> connections;
	double distance;
};

struct SearchPoint {
	RoadPoint* startingPoint;
	RoadPoint* currentPoint;
	std::vector<Connection*> connections;
	double distance;
	double speed;
	double energy;
};

#endif // DATASTRUCTURES_H

#ifndef DATASTRUCTURES_H
#define DATASTRUCTURES_H

#include <vector>


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
};

struct Route {
	RoadPoint* startingPoint;
	std::vector<Connection*> connections;
	double distance;
};

#endif // DATASTRUCTURES_H

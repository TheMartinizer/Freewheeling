#ifndef DATASTRUCTURES_H
#define DATASTRUCTURES_H

#include <vector>
#include <memory>

#include "Constants.h"


struct Connection;

struct RoadPoint
{
	double x, y, z;
	double lat, lon, alt;
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
	
	double getOutputSpeed(double inputSpeed) {
		int bin = (int) (inputSpeed * 3.6) - MINIMUM_SPEED_KMH;
		if (bin < 0 || bin >= MAXIMUM_SPEED_KMH - MINIMUM_SPEED_KMH + 1) {
			return -1;
		}
		
		return speeds[bin];
	}
};

#endif // DATASTRUCTURES_H

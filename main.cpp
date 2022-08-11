#include <iostream>
#include <float.h>
#include <algorithm>
#include <stdio.h>
#include <cmath>
#include <chrono>
#include <unordered_set>

#include "ogrsf_frmts.h"

#include "DataStructures.h"
#include "PointMap.h"

#define DRAG_COEFFICIENT 0.006
#define TIME_STEP 0.1
#define GRAVITY_CONSTANT 9.81
#define MINIMUM_SPEED_KMH 3
#define MAXIMUM_SPEED_KMH 30

using namespace std;

void recursiveRouteSearch(
		Route &route,
		vector<Connection*> &connections,
		RoadPoint* startPoint,
		RoadPoint* previousPoint,
		RoadPoint* currentPoint,
		double currentSpeed,
		double currentDistance,
		unordered_set<RoadPoint*> &startingPointsSet,
		vector<RoadPoint*> &startingPoints,
		unordered_set<RoadPoint*> &visitedPoints
		) {

	// If we reached this point from another point, and it turns out to be something we consider a starting point
	// we can delete it. There's no point to start from here since we can reach here from somewhere else
	if (currentPoint != startPoint && startingPointsSet.find(currentPoint) != startingPointsSet.end()) {
		startingPointsSet.erase(currentPoint);
		auto startingPointIndex = find(startingPoints.begin(), startingPoints.end(), currentPoint);
		startingPoints.erase(startingPointIndex);
	}

	// If there is no way to continue from this point, it is a route
	bool routeStopped = true;

	// Calculate for each possible connection
	for (Connection *connection : currentPoint->connections) {
		RoadPoint* nextPoint = connection->connectedPoint;
		// If this is a connection back to the point we just came from, stop
		if (nextPoint == previousPoint) {
			continue;
		}

		if (visitedPoints.find(nextPoint) != visitedPoints.end()) {
			continue;
		}

		// Check if we can overcome the next connection with the speed we've built
		// Do this through a bit of numerical integration
		double distance = 0;
		double speed = currentSpeed;

		while (distance < connection->horizontalDistance && speed >= MINIMUM_SPEED_KMH / 3.6) {
			distance += speed * TIME_STEP;
			speed += (-GRAVITY_CONSTANT * connection->sinSlope - DRAG_COEFFICIENT * speed * speed) * TIME_STEP;
		}

		if (speed < MINIMUM_SPEED_KMH / 3.6) {
			continue;
		}

		if (speed > MAXIMUM_SPEED_KMH / 3.6) {
			speed = MAXIMUM_SPEED_KMH / 3.6;
		}

		routeStopped = false;
		connections.push_back(connection);
		visitedPoints.insert(currentPoint);
		recursiveRouteSearch(route, connections, startPoint, currentPoint, nextPoint, speed, currentDistance + connection->horizontalDistance, startingPointsSet, startingPoints, visitedPoints);
		visitedPoints.erase(currentPoint);
		connections.pop_back();
	}

	// If the route stopped here, add it to the list
	if (routeStopped & currentDistance > route.distance) {
		OGRSpatialReference source, target;

		source.importFromEPSG(32633);
		target.importFromEPSG(4326);

		OGRCoordinateTransformation *transform = OGRCreateCoordinateTransformation(&source, &target);

		route.distance = currentDistance;
		route.connections = connections;
		double x, y;
		x = startPoint->x;
		y = startPoint->y;
		transform->Transform(1, &x, &y);

		printf("\n\n\n\n");
		printf("LINESTRING (%.6f %.6f", y, x);
		for (Connection* connection : connections) {
			x = connection->connectedPoint->x;
			y = connection->connectedPoint->y;
			transform->Transform(1, &x, &y);
			printf(", %.6f %.6f", y, x);
		}
		printf(")\n");
		cout << flush;
	}
}

Route findLongestRouteFromPoint(RoadPoint* startingPoint, vector<RoadPoint*> &startingPoints) {
	Route route;
	route.distance = 0;

	vector<Connection*> connections;

	unordered_set<RoadPoint*> startingPointsSet;
	for (RoadPoint *roadPoint : startingPoints) {
		startingPointsSet.insert(roadPoint);
	}

	unordered_set<RoadPoint*> visitedPoints;

	recursiveRouteSearch(route, connections, startingPoint, NULL, startingPoint, MINIMUM_SPEED_KMH+2 / 3.6, 0, startingPointsSet, startingPoints, visitedPoints);

	return route;
}



// Used when sorting the list of local maxima, so we start with the very highest point
bool comparePointHeights(RoadPoint* point1, RoadPoint* point2) {
	return point1->z > point2->z;
}

int main()
{
	// Load GDAL
	GDALAllRegister();

	GDALDataset *dataset;
	// Currently, the data file is hardcoded. 0301Elveg2.0.gml maps all the roads in Oslo, and is available from
	// kartkatalog.geonorge.no
	dataset = (GDALDataset*) GDALOpenEx("0301Elveg2.0.gml", GDAL_OF_VECTOR, NULL, NULL, NULL);

	if (!dataset) {
		printf("Could not open file\n");
		exit(1);
	}

	OGRLayer *veger;
	// Veglenke is the feature layer in the ElVeg2.0 dataset that contains all the roads.
	// Veglenke means "road chain" in Norwegian
	veger = dataset->GetLayerByName("Veglenke");

	if (!veger) {
		printf("Could not open layer Veglenke\n");
		exit(2);
	}

	// Next, go through all the features that exist in the layer, and find the maximum and minimum values of their coordinates
	int numberOfFeatures = 0;
	double minimumX = DBL_MAX;
	double maximumX = DBL_MIN;
	double minimumY = DBL_MAX;
	double maximumY = DBL_MIN;
	double minimumZ = DBL_MAX;
	double maximumZ = DBL_MIN;

	std::cout << "Loading features from file..." << std::endl << std::flush;
	for (auto& feature : veger) {
		OGRGeometry *geometry = feature->GetGeometryRef();
		if (wkbFlatten(geometry->getGeometryType()) == wkbLineString) {
			numberOfFeatures++;

			OGRLineString* lineString = geometry->toLineString();
			int numberOfPoints = lineString->getNumPoints();
			for (int i = 0; i < numberOfPoints; i++) {
				OGRPoint point;
				lineString->getPoint(i, &point);

				double x = point.getX();
				double y = point.getY();
				double z = point.getZ();

				minimumX = min(x, minimumX);
				maximumX = max(x, maximumX);
				minimumY = min(y, minimumY);
				maximumY = max(y, maximumY);
				minimumZ = min(z, minimumZ);
				maximumZ = max(z, maximumZ);
			}
		}
	}

	// Make it easier to look up points in order to find connections
	PointMap pointMap(minimumX, maximumX, minimumY, maximumY, minimumZ, maximumZ, 1000, 0.2);

	int counter = 0;
	int points = 0;

	auto lastPrintTime = chrono::system_clock::now();

	for (auto& feature: veger) {
		OGRGeometry *geometry = feature->GetGeometryRef();
		if (wkbFlatten(geometry->getGeometryType()) == wkbLineString) {
			counter++;

			auto now = chrono::system_clock::now();
			if (chrono::duration_cast<chrono::milliseconds>(now - lastPrintTime).count() > 200) {
				printf("\rMapping roads. Currently on feature %d of %d", counter, numberOfFeatures);
				cout << flush;
				lastPrintTime = now;
			}

			OGRLineString* lineString = geometry->toLineString();
			int numberOfPoints = lineString->getNumPoints();
			for (int i = 0; i < numberOfPoints; i++) {
				points++;

				OGRPoint point;
				lineString->getPoint(i, &point);
				RoadPoint* currentRoadPoint = pointMap.getPoint(point.getX(), point.getY(), point.getZ());

				auto currentStart = currentRoadPoint->connections.begin();
				auto currentEnd = currentRoadPoint->connections.end();

				if (i < numberOfPoints - 1) {
					OGRPoint nextPoint;
					lineString->getPoint(i+1, &nextPoint);

					RoadPoint* nextRoadPoint = pointMap.getPoint(nextPoint.getX(), nextPoint.getY(), nextPoint.getZ());

					auto nextStart = nextRoadPoint->connections.begin();
					auto nextEnd = nextRoadPoint->connections.end();

					double distance = sqrt(pow(abs(nextPoint.getX()-point.getX()), 2) + pow(abs(nextPoint.getY()-point.getY()),2));
					double heightDifference = nextPoint.getZ() - point.getZ();

					bool connectionFromCurrentToNext = false;
					bool connectionFromNextToCurrent = false;

					for (auto it = currentStart; it != currentEnd; ++it) {
						if ((*it)->connectedPoint == nextRoadPoint) {
							connectionFromCurrentToNext = true;
							break;
						}
					}

					for (auto it = nextStart; it != nextEnd; ++it) {
						if ((*it)->connectedPoint == currentRoadPoint) {
							connectionFromNextToCurrent = true;
							break;
						}
					}

					if(!connectionFromCurrentToNext) {
						Connection *connection = new Connection;
						connection->connectedPoint = nextRoadPoint;
						connection->horizontalDistance = distance;
						connection->heightDifference = heightDifference;
						connection->sinSlope = sin(atan2(heightDifference, distance));
						currentRoadPoint->connections.push_back(connection);
					}

					if(!connectionFromNextToCurrent) {
						Connection *connection = new Connection;
						connection->connectedPoint = currentRoadPoint;
						connection->horizontalDistance = distance;
						connection->heightDifference = -heightDifference;
						connection->sinSlope = sin(atan2(-heightDifference, distance));
						nextRoadPoint->connections.push_back(connection);
					}
				}
			}
		}
	}

	printf("\rMapping roads. Currently on feature %d of %d\n", numberOfFeatures, numberOfFeatures);
	printf("Finding possible start points...");
	vector<RoadPoint*> startPoints;

	double rollingSlope = DRAG_COEFFICIENT * pow(MINIMUM_SPEED_KMH / 3.6, 2) / GRAVITY_CONSTANT;

	for (RoadPoint* candidate : pointMap.getAllPoints()) {
		bool canRollFrom = false;
		bool canRollTo = false;
		for (Connection *connection : candidate->connections) {
			if (connection->sinSlope > rollingSlope) {
				canRollTo = true;
				break;
			}

			if (connection->sinSlope < -rollingSlope) {
				canRollFrom = true;
			}
		}

		if (canRollFrom && !canRollTo) {
			startPoints.push_back(candidate);
		}
	}

	printf("\rFound %d possible start points\n", startPoints.size());

	// Sort maxima by height
	sort(startPoints.begin(), startPoints.end(), comparePointHeights);

	int pointCounter = 0;
	while (pointCounter < startPoints.size()) {
		RoadPoint* start = startPoints.at(pointCounter);
		pointCounter++;
		Route route = findLongestRouteFromPoint(start, startPoints);
		printf("Found route of length %.0f m from point %d\n", route.distance, pointCounter);
		cout << flush;
	}

}

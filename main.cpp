#include <iostream>
#include <float.h>
#include <algorithm>
#include <stdio.h>
#include <cmath>
#include <chrono>
#include <unordered_map>
#include <unordered_set>

#include "ogrsf_frmts.h"

#include "DataStructures.h"
#include "PointMap.h"
#include "Constants.h"

using namespace std;

bool compareLengths(Route route1, Route route2) {
	return route1.distance > route2.distance;
}

vector<Route> findRoutesFromPoint(shared_ptr<RoadPoint> startingPoint, vector<shared_ptr<RoadPoint>> &startingPoints) {
	unordered_map<shared_ptr<RoadPoint>, Route> routes;
	unordered_map<shared_ptr<RoadPoint>, SearchPoint> searchPoints;
	unordered_set<shared_ptr<RoadPoint>> visited;

	double initialSpeed = MINIMUM_SPEED_KMH / 3.6;
	double initialEnergy = startingPoint->z*GRAVITY_CONSTANT + 0.5*pow(initialSpeed, 2);

	SearchPoint initialSearchPoint;

	initialSearchPoint.startingPoint = startingPoint;
	initialSearchPoint.currentPoint = startingPoint;
	initialSearchPoint.distance = 0;
	initialSearchPoint.speed = initialSpeed;
	initialSearchPoint.energy = initialEnergy;

	searchPoints[startingPoint] = initialSearchPoint;

	while (searchPoints.size() > 0) {
		// Find the SearchPoint with the highest current total energy
		double highestEnergy = 0;
		shared_ptr<RoadPoint> bestSearchPointKey;

		for (auto const& keyValuePair : searchPoints) {
			if (keyValuePair.second.energy > highestEnergy) {
				highestEnergy = keyValuePair.second.energy;
				bestSearchPointKey = keyValuePair.first;
			}
		}

		// Now that we've found the best point, remove it from the list so we won't use it further
		SearchPoint searchPoint = searchPoints[bestSearchPointKey];
		searchPoints.erase(bestSearchPointKey);
		visited.insert(bestSearchPointKey);

		double speedKmh = searchPoint.speed * 3.6;
		int speedRounded = (int) speedKmh;
		int speedBin = speedRounded - MINIMUM_SPEED_KMH;

		// Look through this search point's connections
		bool routeContinues = false;
		for (shared_ptr<Connection> connection : searchPoint.currentPoint->connections) {
			shared_ptr<RoadPoint> nextPoint = connection->connectedPoint;

			if (visited.find(nextPoint) != visited.end()) {
				continue;
			}

			double finalSpeed = connection->speeds[speedBin];
			double finalEnergy = nextPoint->z*GRAVITY_CONSTANT + 0.5*pow(finalSpeed,2);

			if (finalSpeed < MINIMUM_SPEED_KMH / 3.6) {
				continue;
			}
			routeContinues = true;

			SearchPoint nextSearchPoint;

			nextSearchPoint.startingPoint = startingPoint;
			nextSearchPoint.currentPoint = nextPoint;
			nextSearchPoint.connections = searchPoint.connections;
			nextSearchPoint.connections.push_back(connection);

			nextSearchPoint.speed = finalSpeed;
			nextSearchPoint.distance = searchPoint.distance + connection->horizontalDistance;

			if (searchPoints.count(nextPoint)) {
				if (searchPoints[nextPoint].energy < finalEnergy) {
					searchPoints[nextPoint] = nextSearchPoint;
				}
			} else {
				searchPoints[nextPoint] = nextSearchPoint;

				// Check if this is one of the starting points, and if so, erase it
				startingPoints.erase(remove(startingPoints.begin(), startingPoints.end(), nextPoint), startingPoints.end());
			}
		}

		if (!routeContinues) {
			Route route;
			route.startingPoint = startingPoint;
			route.connections = searchPoint.connections;
			route.distance = searchPoint.distance;

			// Check if this route is longer than the previous route stored for this point
			if (routes.count(searchPoint.currentPoint)) {
				if (routes[searchPoint.currentPoint].distance < route.distance) {
					routes[searchPoint.currentPoint] = route;
				}
			} else {
				routes[searchPoint.currentPoint] = route;
			}

			//printf("\rFound %d routes from current point", routes.size());
			cout << flush;
		}
	}

	vector<Route> finalRoutes;
	for (auto const& entries : routes) {
		finalRoutes.push_back(entries.second);
	}

	sort(finalRoutes.begin(), finalRoutes.end(), compareLengths);

	return finalRoutes;
}



// Used when sorting the list of local maxima, so we start with the very highest point
bool comparePointHeights(shared_ptr<RoadPoint> point1, shared_ptr<RoadPoint> point2) {
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

	PointMap pointMap(dataset);
	printf("\nFinding possible start points...");
	vector<shared_ptr<RoadPoint>> startPoints;

	double rollingSlope = DRAG_COEFFICIENT * pow(MINIMUM_SPEED_KMH / 3.6, 2) / GRAVITY_CONSTANT;

	for (shared_ptr<RoadPoint> candidate : pointMap.getAllPoints()) {
		bool canRollFrom = false;
		bool canRollTo = false;
		for (shared_ptr<Connection> connection : candidate->connections) {
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

	printf("\rFound %zu possible start points\n", startPoints.size());

	// Sort maxima by height
	sort(startPoints.begin(), startPoints.end(), comparePointHeights);

	vector<Route> allLongest;

	int pointCounter = 0;
	OGRSpatialReference source, target;

	source.importFromEPSG(32633);
	target.importFromEPSG(4326);

	OGRCoordinateTransformation *transform = OGRCreateCoordinateTransformation(&source, &target);

	while (pointCounter < startPoints.size()) {
		shared_ptr<RoadPoint> start = startPoints.at(pointCounter);
		pointCounter++;

		// Only bother to do the first 1000 points
		if (pointCounter > 1000) {
			break;
		}

		// Below sea level is probably not going to be a good start point
		if (start->z <= 0) {
			break;
		}

		vector<Route> routes = findRoutesFromPoint(start, startPoints);
		// Find a set of different routes going to different places
		vector<Route> chosenRoutes;
		double longestDistance = routes.at(0).distance;
		for (Route possibleRoute : routes) {
			// Make sure the route is long enough
			if (possibleRoute.distance < 0.5*longestDistance) {
				break;
			}
			// Make sure the end is far enough away from all other points
			shared_ptr<RoadPoint> lastPoint = possibleRoute.connections.back()->connectedPoint;
			bool farAway = true;
			for (Route otherRoute : chosenRoutes) {
				shared_ptr<RoadPoint> otherLastPoint = otherRoute.connections.back()->connectedPoint;

				double distance = sqrt(pow(lastPoint->x - otherLastPoint->x,2)+pow(lastPoint->y - otherLastPoint->y, 2));
				if (distance < 150) {
					farAway = false;
					break;
				}
			}

			if (farAway) {
				chosenRoutes.push_back(possibleRoute);
			}
		}

		for (Route chosenRoute : chosenRoutes) {
			allLongest.push_back(chosenRoute);
		}

		printf("\rSearched through %d of %zu starting points", pointCounter, startPoints.size());

		cout << flush;
	}
	printf("\n");

	sort(allLongest.begin(), allLongest.end(), compareLengths);

	FILE *filePtr = fopen("result.txt", "w");

	fprintf(filePtr, "MULTILINESTRING(");

	int routeCounter = 0;
	for (Route route : allLongest) {
		routeCounter++;
		double x, y;
		x = route.startingPoint->x;
		y = route.startingPoint->y;

		transform->Transform(1,&x,&y);

		fprintf(filePtr, "( %.6f %.6f", y, x);
		for (shared_ptr<Connection> connection : route.connections) {
			x = connection->connectedPoint->x;
			y = connection->connectedPoint->y;
			transform->Transform(1,&x,&y);
			fprintf(filePtr, ", %.6f %.6f", y, x);
		}
		fprintf(filePtr, ")");

		if (routeCounter >= 100) {
			fprintf(filePtr, ")");
			break;
		} else {
			fprintf(filePtr, ",");
		}
	}
	fclose(filePtr);
}

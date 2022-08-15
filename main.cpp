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

vector<Route> findRoutesFromPoint(RoadPoint* startingPoint, vector<RoadPoint*> &startingPoints) {
	unordered_map<RoadPoint*, Route> routes;
	unordered_map<RoadPoint*, SearchPoint> searchPoints;
	unordered_set<RoadPoint*> visited;

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
		RoadPoint* bestSearchPointKey;

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
		int speedRounded = round(speedKmh);
		int speedBin = speedRounded - MINIMUM_SPEED_KMH;

		// Look through this search point's connections
		bool routeContinues = false;
		for (Connection* connection : searchPoint.currentPoint->connections) {
			RoadPoint* nextPoint = connection->connectedPoint;

			if (visited.find(nextPoint) != visited.end()) {
				continue;
			}

			double finalSpeed = connection->speeds[speedBin] + (speedKmh - speedRounded) / 3.6;
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
						connection->cosSlope = cos(atan2(heightDifference, distance));

						for (int j = 0; j < MAXIMUM_SPEED_KMH - MINIMUM_SPEED_KMH + 1; j++) {
							double distance = 0;
							double speed = (MINIMUM_SPEED_KMH + j) / 3.6;

							while (distance < connection->horizontalDistance/connection->cosSlope && speed >= MINIMUM_SPEED_KMH / 3.6) {
								distance += speed * TIME_STEP;
								speed += (-GRAVITY_CONSTANT * connection->sinSlope - DRAG_COEFFICIENT * speed * speed) * TIME_STEP;
							}

							if (speed < MINIMUM_SPEED_KMH / 3.6) {
								speed = -1;
							}

							if (speed > MAXIMUM_SPEED_KMH / 3.6) {
								speed = MAXIMUM_SPEED_KMH / 3.6;
							}

							connection->speeds[j] = speed;
						}

						currentRoadPoint->connections.push_back(connection);
					}

					if(!connectionFromNextToCurrent) {
						Connection *connection = new Connection;
						connection->connectedPoint = currentRoadPoint;
						connection->horizontalDistance = distance;
						connection->heightDifference = -heightDifference;
						connection->sinSlope = sin(atan2(-heightDifference, distance));
						connection->cosSlope = cos(atan2(-heightDifference, distance));

						for (int j = 0; j < MAXIMUM_SPEED_KMH - MINIMUM_SPEED_KMH + 1; j++) {
							double distance = 0;
							double speed = (MINIMUM_SPEED_KMH + j) / 3.6;

							while (distance < connection->horizontalDistance/connection->cosSlope && speed >= MINIMUM_SPEED_KMH / 3.6) {
								distance += speed * TIME_STEP;
								speed += (-GRAVITY_CONSTANT * connection->sinSlope - DRAG_COEFFICIENT * speed * speed) * TIME_STEP;
							}

							if (speed < MINIMUM_SPEED_KMH / 3.6) {
								speed = -1;
							}

							if (speed > MAXIMUM_SPEED_KMH / 3.6) {
								speed = MAXIMUM_SPEED_KMH / 3.6;
							}

							connection->speeds[j] = speed;
						}

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

	vector<Route> allLongest;

	int pointCounter = 0;
	OGRSpatialReference source, target;

	source.importFromEPSG(32633);
	target.importFromEPSG(4326);

	OGRCoordinateTransformation *transform = OGRCreateCoordinateTransformation(&source, &target);

	while (pointCounter < startPoints.size()) {
		RoadPoint* start = startPoints.at(pointCounter);
		pointCounter++;

		// Below sea level is probably not going to be a good start point
		if (start->z <= 0) {
			break;
		}

		vector<Route> routes = findRoutesFromPoint(start, startPoints);
		Route longest = routes.at(0);
		allLongest.push_back(longest);
		sort(allLongest.begin(), allLongest.end(), compareLengths);
		while (allLongest.size() > 10) {
			allLongest.pop_back();
		}

		printf("\rSearched through %d of %d starting points", pointCounter, startPoints.size());

		cout << flush;
	}

	int routeCounter = 0;
	for (Route route : allLongest) {
		routeCounter++;
		double x, y;
		x = route.startingPoint->x;
		y = route.startingPoint->y;

		transform->Transform(1,&x,&y);

		printf("\nLINESTRING( %.6f %.6f", y, x);
		for (Connection *connection : route.connections) {
			x = connection->connectedPoint->x;
			y = connection->connectedPoint->y;
			transform->Transform(1,&x,&y);
			printf(", %.6f %.6f", y, x);
		}
		printf(")\n");

		if (routeCounter >= 10) {
			break;
		}
	}
}

#include <iostream>
#include <float.h>
#include <algorithm>
#include <stdio.h>
#include <cmath>
#include <chrono>

#include "ogrsf_frmts.h"

using namespace std;

struct Connection;

struct RoadPoint
{
	double x, y, z;
	vector<Connection> connections;
};

struct Connection {
	RoadPoint* connectedPoint;
	double horizontalDistance;
	double heightDifference;
};

class PointMap {
private:
	double minimumX, maximumX, minimumY, maximumY, minimumZ, maximumZ, pointThreshold;
	int numberOfBins;
	map<int, vector<RoadPoint*>*> pointMap;
	vector<RoadPoint*> roadPoints;

public:
	PointMap(double minimumX, double maximumX, double minimumY, double maximumY, double minimumZ, double maximumZ, int numberOfBins, double pointThreshold);

	~PointMap() {
		for (RoadPoint* roadPoint : roadPoints) {
			delete roadPoint;
		}

		for (auto mapEntry : pointMap) {
			delete mapEntry.second;
		}
	}

	RoadPoint* getPoint(double x, double y, double z) {
		double xValue = ((x - minimumX) * numberOfBins / (maximumX - minimumX));
		double yValue = ((y - minimumY) * numberOfBins / (maximumY - minimumY));
		double zValue = ((z - minimumZ) * numberOfBins / (maximumZ - minimumZ));

		vector<int> xValues;
		vector<int> yValues;
		vector<int> zValues;

		double xThreshold = pointThreshold * numberOfBins / (maximumX - minimumX);
		xValues.push_back((int) xValue);

		if (xValue - (int) xValue <= xThreshold) {
			xValues.push_back((int) xValue - 1);
		}

		if ((int) xValue + 1 - xValue <= xThreshold) {
			xValues.push_back((int) xValue + 1);
		}


		double yThreshold = pointThreshold * numberOfBins / (maximumY - minimumY);
		yValues.push_back((int) yValue);

		if (yValue - (int) yValue <= yThreshold) {
			yValues.push_back((int) yValue - 1);
		}

		if ((int) yValue + 1 - yValue <= yThreshold) {
			yValues.push_back((int) yValue + 1);
		}

		double zThreshold = pointThreshold * numberOfBins / (maximumZ - minimumZ);
		zValues.push_back((int) zValue);

		if (zValue - (int) zValue <= zThreshold) {
			zValues.push_back((int) zValue - 1);
		}

		if ((int) zValue + 1 - zValue <= zThreshold) {
			zValues.push_back((int) zValue + 1);
		}

		for (int xBin : xValues) {
			for (int yBin : yValues) {
				for (int zBin : zValues) {
					int bin = (xBin * numberOfBins + yBin) * numberOfBins + zBin;
					if (pointMap.find(bin) == pointMap.end()) {
						pointMap[bin] = new vector<RoadPoint*>;
					}

					for (RoadPoint* roadPoint : *pointMap[bin]) {
						if (abs(roadPoint->x - x) <= pointThreshold
								&& abs(roadPoint->y - y) <= pointThreshold
								&& abs(roadPoint->z - z) <= pointThreshold) {
							return roadPoint;
						}
					}
				}
			}
		}

		// If no point was found, generate a new one
		RoadPoint* newPoint = new RoadPoint;
		newPoint->x = x;
		newPoint->y = y;
		newPoint->z = z;

		int bin = ((int) xValue * numberOfBins + (int) yValue) * numberOfBins + (int) zValue;
		pointMap[bin]->push_back(newPoint);
		roadPoints.push_back(newPoint);

		return newPoint;
	}

	vector<RoadPoint*> getAllPoints() {
		return roadPoints;
	}
};

bool comparePointHeights(RoadPoint* point1, RoadPoint* point2) {
	return point1->z > point2->z;
}

int main()
{
	GDALAllRegister();

	GDALDataset *dataset;
	dataset = (GDALDataset*) GDALOpenEx("/home/martin/dev/python/sykkelrulle/0301Elveg2.0.gml", GDAL_OF_VECTOR, NULL, NULL, NULL);

	if (!dataset) {
		printf("Could not open file\n");
		exit(1);
	}

	OGRLayer *veger;
	veger = dataset->GetLayerByName("Veglenke");

	if (!veger) {
		printf("Could not open layer Veglenke\n");
		exit(2);
	}

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
						if (it->connectedPoint == nextRoadPoint) {
							connectionFromCurrentToNext = true;
							break;
						}
					}

					for (auto it = nextStart; it != nextEnd; ++it) {
						if (it->connectedPoint == currentRoadPoint) {
							connectionFromNextToCurrent = true;
							break;
						}
					}

					if(!connectionFromCurrentToNext) {
						Connection connection;
						connection.connectedPoint = nextRoadPoint;
						connection.horizontalDistance = distance;
						connection.heightDifference = heightDifference;
						currentRoadPoint->connections.push_back(connection);
					}

					if(!connectionFromNextToCurrent) {
						Connection connection;
						connection.connectedPoint = currentRoadPoint;
						connection.horizontalDistance = distance;
						connection.heightDifference = -heightDifference;
						currentRoadPoint->connections.push_back(connection);
						nextRoadPoint->connections.push_back(connection);
					}
				}
			}
		}
	}

	printf("\rMapping roads. Currently on feature %d of %d\n", numberOfFeatures, numberOfFeatures);
	printf("Finding local maxima...");
	vector<RoadPoint*> maxima;

	for (RoadPoint* candidate : pointMap.getAllPoints()) {
		double height = candidate->z;
		bool highest = true;
		for (Connection connection : candidate->connections) {
			RoadPoint* roadPoint = connection.connectedPoint;
			double connectionHeight = roadPoint->z;

			if (connectionHeight > height) {
				highest = false;
				break;
			}
		}

		if (highest) {
			maxima.push_back(candidate);
		}
	}

	printf("\rFound %d local maxima\n", maxima.size());

	// Sort maxima by height
	sort(maxima.begin(), maxima.end(), comparePointHeights);

	for (RoadPoint* start : maxima) {

	}

}

PointMap::PointMap(double minimumX, double maximumX, double minimumY, double maximumY, double minimumZ, double maximumZ, int numberOfBins, double pointThreshold) : minimumX(minimumX),
	maximumX(maximumX),
	minimumY(minimumY),
	maximumY(maximumY),
	minimumZ(minimumZ),
	maximumZ(maximumZ),
	numberOfBins(numberOfBins),
	pointThreshold(pointThreshold)
{}

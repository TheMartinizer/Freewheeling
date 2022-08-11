#include "PointMap.h"

#include <algorithm>

PointMap::PointMap(
		double minimumX,
		double maximumX,
		double minimumY,
		double maximumY,
		double minimumZ,
		double maximumZ,
		int numberOfBins,
		double pointThreshold
) :
	minimumX(minimumX),
	maximumX(maximumX),
	minimumY(minimumY),
	maximumY(maximumY),
	minimumZ(minimumZ),
	maximumZ(maximumZ),
	numberOfBins(numberOfBins),
	pointThreshold(pointThreshold)
{

};

PointMap::~PointMap() {
	for (RoadPoint* roadPoint : roadPoints) {
		delete roadPoint;
	}

	for (auto mapEntry : pointMap) {
		delete mapEntry.second;
	}
}

RoadPoint* PointMap::getPoint(double x, double y, double z) {
	double xValue = ((x - minimumX) * numberOfBins / (maximumX - minimumX));
	double yValue = ((y - minimumY) * numberOfBins / (maximumY - minimumY));
	double zValue = ((z - minimumZ) * numberOfBins / (maximumZ - minimumZ));

	std::vector<int> xValues;
	std::vector<int> yValues;
	std::vector<int> zValues;

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
					pointMap[bin] = new std::vector<RoadPoint*>;
				}

				for (RoadPoint* roadPoint : *pointMap[bin]) {
					if (std::abs(roadPoint->x - x) <= pointThreshold
							&& std::abs(roadPoint->y - y) <= pointThreshold
							&& std::abs(roadPoint->z - z) <= pointThreshold) {
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

std::vector<RoadPoint*> PointMap::getAllPoints() {
	return roadPoints;
}

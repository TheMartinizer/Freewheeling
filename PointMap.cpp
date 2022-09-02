#include "PointMap.h"

#include <algorithm>
#include <unordered_map>
#include <iostream>
#include <limits>
#include <chrono>
#include <cfloat>

void simulateSlope(std::shared_ptr<Connection> connection) {
	for (int initialSpeed = MINIMUM_SPEED_KMH; initialSpeed <= MAXIMUM_SPEED_KMH; initialSpeed++) {
		double speed = initialSpeed / 3.6;
		double distance = 0;

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

		connection->speeds[initialSpeed - MINIMUM_SPEED_KMH] = speed;
	}
}

PointMap::PointMap() : srcEpsg(0), transformation(nullptr) {
	minimumX = DBL_MAX;
	maximumX = -DBL_MAX;
	minimumY = DBL_MAX;
	maximumY = -DBL_MAX;
	minimumZ = DBL_MAX;
	maximumZ = -DBL_MAX;
}

void PointMap::loadDataset(GDALDataset *dataset) {
	loadDataset(dataset, [](Progress progress){printf("\r%s: %d%%", progress.message.c_str(), progress.percent); std::cout << std::flush;});
}

void PointMap::loadDataset(GDALDataset* dataset, std::function<void(Progress progress)> progressCallback) {
	// First, find the speed limits
	OGRLayer *speedLimits;
	speedLimits = dataset->GetLayerByName("Fartsgrense");

	std::unordered_map<int, int> linkIdToSpeedLimitMap;

	if (progressCallback) {
		Progress currentProgress;
		currentProgress.message = "Finding speed limits";
		currentProgress.percent = 0;
		
		progressCallback(currentProgress);
	}
	for (auto& feature : speedLimits) {
		int idField = feature->GetFieldIndex("lineærposisjon|LineærPosisjonStrekning|lenkesekvens|Identifikasjon|lokalId");
		int speedLimitField = feature->GetFieldIndex("fartsgrenseVerdi");

		int id = feature->GetFieldAsInteger(idField);
		int speedLimit = feature->GetFieldAsInteger(speedLimitField);

		linkIdToSpeedLimitMap[id] = speedLimit;
	}

	// Find the roads
	OGRLayer *roads;
	// Veglenke is the feature layer in the ElVeg2.0 dataset that contains all the roads.
	// Veglenke means "road chain" in Norwegian
	roads = dataset->GetLayerByName("Veglenke");

	if (progressCallback) {
		Progress currentProgress;
		currentProgress.message = "Finding extents of road network";
		currentProgress.percent = 0;
		progressCallback(currentProgress);
	}
	
	minimumX = DBL_MAX;
	minimumY = DBL_MAX;
	minimumZ = DBL_MAX;
	maximumX = -DBL_MAX;
	maximumY = -DBL_MAX;
	maximumZ = -DBL_MAX;

	// Find the extents that the PointMap needs to extend to
	int numberOfRoads = 0;
	for (auto& road : roads) {
		// Assume all geometries are Linestrings, which I think they will be
		OGRLineString *geometry = road->GetGeometryRef()->toLineString();
		for (auto& point : geometry) {
			minimumX = std::min(point.getX(), minimumX);
			minimumY = std::min(point.getY(), minimumY);
			minimumZ = std::min(point.getZ(), minimumZ);

			maximumX = std::max(point.getX(), maximumX);
			maximumY = std::max(point.getY(), maximumY);
			maximumZ = std::max(point.getZ(), maximumZ);
		}
		numberOfRoads++;
	}
	
	minimumX -= 1;
	minimumY -= 1;
	minimumZ -= 1;
	maximumX += 1;
	maximumY += 1;
	maximumZ += 1;
	
	rebuildPointMap();

	// Next, create the point map with its connections
	if (progressCallback) {
		Progress currentProgress;
		currentProgress.message = "Creating graph of road network";
		currentProgress.percent = 0;
		progressCallback(currentProgress);
	}
	auto lastUpdateTime = std::chrono::system_clock::now();
	int counter = 0;
	for (auto& road : roads) {
		if (progressCallback) {
			counter++;
			auto now = std::chrono::system_clock::now();
			if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastUpdateTime).count() > 200) {
				Progress currentProgress;
				currentProgress.message = "Creating graph of road network";
				currentProgress.percent = (int) (counter * 100.0 / numberOfRoads);
				currentProgress.loadedRoads = loadedRoads;
				progressCallback(currentProgress);
				lastUpdateTime = now;
			}
		}

		// First, check if speed limit is too high
		int roadIdFieldIndex = road->GetFieldIndex("lenkesekvens|Lenkesekvensreferanse|identifikasjon|Identifikasjon|lokalId");
		int roadId = road->GetFieldAsInteger(roadIdFieldIndex);
		// If no speed limit, just assume it is a tiny road and cyclable
		if (linkIdToSpeedLimitMap.count(roadId)) {
			int speedLimit = linkIdToSpeedLimitMap[roadId];
			if (speedLimit > MAXIMUM_ROAD_SPEED_LIMIT_KMH) {
				continue;
			}
		}

		// If speed limit is not too high, go through the points on the road
		// Find out which directions it is possible to move
		bool forwardsPossible = false;
		bool backwardsPossible = false;
		bool forwardsBikelane = false;
		bool backwardsBikelane = false;

		int typeIndex = road->GetFieldIndex("typeVeg");
		const char* roadType = road->GetFieldAsString(typeIndex);

		// If this road is a staircase (which it might apparently be), we don't want to bike it
		if (
				strcmp(roadType, "trapp") == 0 
				|| strcmp(roadType, "motorveg") == 0
				|| strcmp(roadType, "motortrafikkveg") == 0
				|| strcmp(roadType, "bilferje") == 0
				|| strcmp(roadType, "passasjerferje") == 0
		) {
			continue;
		}

		if (strcmp(roadType, "gangOgSykkelveg") == 0 || strcmp(roadType, "sykkelveg") == 0) {
			forwardsBikelane = true;
			backwardsBikelane = true;
		}

		// If there are odd numbered lanes, they are going forward. Even numbered lanes are going backwards
		// Lanes marked with an S at the end are bike lanes

		// The token # is used delimit lanes

		int lanesIndex = road->GetFieldIndex("feltoversikt");
		std::string lanes = road->GetFieldAsString(lanesIndex);

		size_t pos = 0;
		while (lanes.length() > 0) {
			pos = lanes.find("#");
			if (pos == std::string::npos) {
				pos = lanes.length();
			}

			int laneNumber = atoi(lanes.substr(0,pos).c_str());
			bool forward = (laneNumber % 2) == 1;
			bool bikeLane = lanes.substr(0,pos).find("S") != std::string::npos;

			if (forward) {
				forwardsPossible = true;
				if (bikeLane) {
					forwardsBikelane = true;
				}
			} else {
				backwardsPossible = true;
				if (bikeLane) {
					backwardsBikelane = true;
				}
			}

			lanes.erase(0, pos+1);
		}

		// Go through all the points and create road points and connections
		OGRLineString *lineString = road->GetGeometryRef()->toLineString();
		std::vector<std::shared_ptr<RoadPoint>> loadedRoad;
		for (int i = 0; i < lineString->getNumPoints() - 1; i++) {
			OGRPoint thisOgrPoint;
			lineString->getPoint(i, &thisOgrPoint);

			OGRPoint nextOgrPoint;
			lineString->getPoint(i+1, &nextOgrPoint);

			std::shared_ptr<RoadPoint> thisPoint = getPoint(thisOgrPoint.getX(), thisOgrPoint.getY(), thisOgrPoint.getZ(), road->GetGeometryRef()->getSpatialReference());
			std::shared_ptr<RoadPoint> nextPoint = getPoint(nextOgrPoint.getX(), nextOgrPoint.getY(), nextOgrPoint.getZ(), road->GetGeometryRef()->getSpatialReference());
			
			loadedRoad.push_back(thisPoint);
			if (i == lineString->getNumPoints() - 2) {
				loadedRoad.push_back(nextPoint);
			}

			double distance = sqrt(pow(nextPoint->x - thisPoint->x, 2) + pow(nextPoint->y - thisPoint->y, 2));
			double heightDifference = nextPoint->z - thisPoint->z;
			double slope = atan2(heightDifference, distance);
			double sinSlope = sin(slope);
			double cosSlope = cos(slope);

			if (forwardsPossible) {
				std::shared_ptr<Connection> connection(new Connection);

				connection->connectedPoint = nextPoint;
				connection->horizontalDistance = distance;
				connection->heightDifference = heightDifference;
				connection->sinSlope = sinSlope;
				connection->cosSlope = cosSlope;
				connection->hasBikeLane = forwardsBikelane;

				simulateSlope(connection);

				thisPoint->connections.push_back(connection);
			}

			if (backwardsPossible) {
				std::shared_ptr<Connection> connection(new Connection);

				connection->connectedPoint = thisPoint;
				connection->horizontalDistance = distance;
				connection->heightDifference = -heightDifference;
				connection->sinSlope = -sinSlope;
				connection->cosSlope = cosSlope;
				connection->hasBikeLane = backwardsBikelane;

				simulateSlope(connection);

				nextPoint->connections.push_back(connection);
			}
		}
		
		loadedRoads.push_back(loadedRoad);
	}
}

void PointMap::rebuildPointMap() {
	pointMap.clear();
	loadedRoads.clear();
	
	for (std::shared_ptr<RoadPoint> roadPoint : roadPoints) {
		int xBin = (int) ((roadPoint->x - minimumX) * numberOfBins / (maximumX - minimumX));
		int yBin = (int) ((roadPoint->y - minimumY) * numberOfBins / (maximumY - minimumY));
		int zBin = (int) ((roadPoint->z - minimumZ) * numberOfBins / (maximumZ - minimumZ));
		
		if (xBin < 0 || xBin >= NUMBER_OF_BINS || yBin < 0 || yBin >= NUMBER_OF_BINS || zBin < 0 || zBin >= NUMBER_OF_BINS) {
			continue;
		}
		
		int bin = (xBin * numberOfBins + yBin) * numberOfBins + zBin;
		if (pointMap.find(bin) == pointMap.end()) {
			pointMap[bin] = std::shared_ptr<std::vector<std::shared_ptr<RoadPoint>>>(new std::vector<std::shared_ptr<RoadPoint>>);
		}
		
		pointMap[bin]->push_back(roadPoint);
	}
}

std::shared_ptr<RoadPoint> PointMap::getPoint(double x, double y, double z, OGRSpatialReference *ref) {
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
					pointMap[bin] = std::shared_ptr<std::vector<std::shared_ptr<RoadPoint>>>(new std::vector<std::shared_ptr<RoadPoint>>);
				}

				for (std::shared_ptr<RoadPoint> roadPoint : *pointMap[bin]) {
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
	std::shared_ptr<RoadPoint> newPoint = std::shared_ptr<RoadPoint>(new RoadPoint);
	newPoint->x = x;
	newPoint->y = y;
	newPoint->z = z;
	
	// Calculate lat, lon and alt
	double newX = x;
	double newY = y;
	double newZ = z;
	
	if (ref->GetEPSGGeogCS() != srcEpsg) {
		OGRSpatialReference targetReference;
		targetReference.importFromEPSG(4326);
		transformation = OGRCreateCoordinateTransformation(ref, &targetReference);
		
		srcEpsg = ref->GetEPSGGeogCS();
	}
	
	transformation->Transform(1, &newX, &newY, &newZ);
	
	newPoint->lat = newX;
	newPoint->lon = newY;
	newPoint->alt = newZ;

	int bin = ((int) xValue * numberOfBins + (int) yValue) * numberOfBins + (int) zValue;
	pointMap[bin]->push_back(newPoint);
	roadPoints.push_back(newPoint);

	return newPoint;
}

std::vector<std::shared_ptr<RoadPoint>> PointMap::getAllPoints() {
	return roadPoints;
}

std::vector<std::vector<std::shared_ptr<RoadPoint>>> PointMap::getLoadedRoads() {
	return loadedRoads;
}

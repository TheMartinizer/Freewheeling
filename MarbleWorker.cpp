#include "MarbleWorker.h"

#include <iostream>
#include <algorithm>
#include <unordered_set>

#include <QColor>
#include <marble/GeoDataMultiGeometry.h>
#include <marble/GeoDataLineString.h>
#include <marble/GeoDataPoint.h>
#include <marble/GeoDataCoordinates.h>
#include <marble/GeoDataLineStyle.h>
#include <marble/GeoDataStyle.h>
#include <marble/GeoDataPlacemark.h>

using namespace Marble;
MarbleWorker::MarbleWorker(QObject *parent) : QThread(parent) {
}

MarbleWorker::~MarbleWorker() {
	mutex.lock();
	stop = true;
	condition.wakeAll();
	mutex.unlock();
	
	wait();
}

void MarbleWorker::loadData(QString fileToLoad) {
	QMutexLocker locker(&mutex);
	
	this->shouldLoadFile = true;
	this->filename = fileToLoad;
	
	if (!isRunning()) {
		start(LowPriority);
	} else {
		condition.wakeAll();
	}
}

void MarbleWorker::findStartingPoints() {
	QMutexLocker locker(&mutex);
	
	this->shouldFindStartingPoints = true;
	
	if (!isRunning()) {
		start(LowPriority);
	} else {
		condition.wakeAll();
	}
}

void MarbleWorker::receiveClick(float lat, float lon) {
	QMutexLocker locker(&mutex);
	
	this->receivedClick = true;
	this->clickedLat = lat;
	this->clickedLon = lon;
	
	if (!isRunning()) {
		start(LowPriority);
	} else {
		condition.wakeAll();
	}
}

Marble::GeoDataDocument *MarbleWorker::createDocument(Progress progress, int roads) {
	GeoDataMultiGeometry *multiGeometry = new GeoDataMultiGeometry();
	int roadCounter = 0;
	for (std::vector<std::shared_ptr<RoadPoint>> loadedRoad : progress.loadedRoads) {
		roadCounter++;
		if (roadCounter <= roads) {
			continue;
		}
		
		GeoDataLineString *lineString = new GeoDataLineString();
		for (std::shared_ptr<RoadPoint> point : loadedRoad) {
			lineString->append(GeoDataCoordinates(point->lon, point->lat, point->alt, GeoDataCoordinates::Degree));
		}
		multiGeometry->append(lineString);
	}
	
	GeoDataLineStyle lineStyle(QColor(120, 120, 120, 90));
	lineStyle.setPhysicalWidth(5.0);
	
	QSharedPointer<GeoDataStyle> geoDataStyle = QSharedPointer<GeoDataStyle>(new GeoDataStyle());
	geoDataStyle->setLineStyle(lineStyle);
	
	GeoDataPlacemark *routePlacemark = new GeoDataPlacemark();
	routePlacemark->setGeometry(multiGeometry);
	
	routePlacemark->setStyle(geoDataStyle);
	
	GeoDataDocument *document = new GeoDataDocument();
	document->append(routePlacemark);
	
	return document;
}

void MarbleWorker::addRouteGraphsToDocument(std::vector<std::shared_ptr<RouteGraph>> routeGraphs, GeoDataDocument *document, int number) {
	
	int counter = 0;
	for (std::shared_ptr<RouteGraph> savedRouteGraph : routeGraphs) {
		counter++;
		if (counter > number && number >= 0) {
			break;
		}
		
		GeoDataPlacemark *placemark = new GeoDataPlacemark();
		placemark->setGeometry(new GeoDataPoint(savedRouteGraph->startingPoint->lon, savedRouteGraph->startingPoint->lat, savedRouteGraph->startingPoint->alt, GeoDataCoordinates::Degree));
		placemark->setDescription(QString("%1 %2").arg(savedRouteGraph->farthestEndpoint->lat, 0, 'f', 6).arg(savedRouteGraph->farthestEndpoint->lon, 0, 'f', 6));
		
		document->append(placemark);
	}
}

void MarbleWorker::addRouteGraphToDocument(std::shared_ptr<RouteGraph> routeGraph, GeoDataDocument *document) {
	std::unordered_set<std::shared_ptr<RoadPoint>> visited;
	std::vector<std::pair<std::shared_ptr<RoadPoint>, std::shared_ptr<RoadPoint>>> toVisit;
	toVisit.push_back(std::pair(routeGraph->startingPoint, nullptr));
	GeoDataMultiGeometry *multi = new GeoDataMultiGeometry();
	
	GeoDataLineString *lineString = new GeoDataLineString();
	
	auto it = toVisit.begin();
	while (it != toVisit.end()) {
		std::shared_ptr<RoadPoint> currentPoint = it->first;
		visited.insert(currentPoint);
		GeoDataCoordinates currentCoordinates(currentPoint->lon, currentPoint->lat, currentPoint->alt, GeoDataCoordinates::Degree);
		
		if (lineString->size() == 0 && it->second != nullptr) {
			GeoDataCoordinates connectedCoordinates(it->second->lon, it->second->lat, it->second->alt, GeoDataCoordinates::Degree);
			lineString->append(connectedCoordinates);
		}
		
		lineString->append(currentCoordinates);
		
		int possibleConnections = 0;
		for (std::shared_ptr<Connection> connection : currentPoint->connections) {
			std::shared_ptr<RoadPoint> connectedPoint = connection->connectedPoint;
			
			if (visited.count(connectedPoint) == 0 && routeGraph->reachablePoints.count(connectedPoint) == 1) {
				possibleConnections++;
				
				it++;
				it = toVisit.insert(it, std::pair(connectedPoint, currentPoint));
				it--;
			}
		}
		
		if (possibleConnections != 1) {
			if (lineString->size() > 1) {
				multi->append(lineString);
			} else {
				delete lineString;
			}
			lineString = new GeoDataLineString();
			
			if (possibleConnections > 0) {
				lineString->append(currentCoordinates);
			}
		}
		
		it++;
	}
	
	if (lineString->size() > 1) {
		multi->append(lineString);
	}
	
	GeoDataPlacemark *placemark = new GeoDataPlacemark();
	placemark->setGeometry(multi);
	
	GeoDataLineStyle lineStyle(QColor(255, 0, 0, 90));
	lineStyle.setPhysicalWidth(5.0);
	
	QSharedPointer<GeoDataStyle> geoDataStyle = QSharedPointer<GeoDataStyle>(new GeoDataStyle());
	geoDataStyle->setLineStyle(lineStyle);
	
	placemark->setStyle(geoDataStyle);
	document->append(placemark);
}

void MarbleWorker::run() {
	forever {
		mutex.lock();
		if (this->stop) {
			return;
		}
				
		bool shouldLoadFile = this->shouldLoadFile;
		QString fileName = this->filename;
		
		this->shouldLoadFile = false;
		this->filename = QString("");
		
		bool shouldFindStartingPoints = this->shouldFindStartingPoints;
		this->shouldFindStartingPoints = false;
		
		bool receivedClick = this->receivedClick;
		this->receivedClick = false;
		double clickedLat = this->clickedLat;
		double clickedLon = this->clickedLon;
		mutex.unlock();

		if (shouldLoadFile) {
			GDALDataset *dataset;
			dataset = (GDALDataset*) GDALOpenEx(fileName.toLocal8Bit().data(), GDAL_OF_VECTOR, NULL, NULL, NULL);
			if (!dataset) {
				continue;
			}
			
			int roads = 0;
			if (!pointMap) {
				pointMap = new PointMap();
			}
			
			pointMap->loadDataset(dataset, [this, &roads](Progress progress) {
				Marble::GeoDataDocument *document = createDocument(progress, roads);
				emit this->mapUpdated(document, true, QString(progress.message.c_str()), progress.percent);
				roads = progress.loadedRoads.size();
				printf("\r%s: %i%%", progress.message.c_str(), progress.percent);
				std::cout << std::flush;
			});
			
			emit mapUpdated(nullptr, false, QString("Finished loading road data"), 100);
		}
		
		if (shouldFindStartingPoints) {
			routeGraphs.clear();
			emit mapUpdated(nullptr, false, QString("Searching for possible starting points"), 0);
			
			// Find possible starting points
			std::vector<std::shared_ptr<RoadPoint>> startingPoints;
			
			// Go through all points found and find the ones that it is possible to go out from,
			// but not into
			for (std::shared_ptr<RoadPoint> possibleStart : pointMap->getAllPoints()) {
				bool canRollOut = false;
				bool canRollIn = false;
				
				for (std::shared_ptr<Connection> connection : possibleStart->connections) {
					if (connection->getOutputSpeed(MINIMUM_SPEED_KMH / 3.6) > 0) {
						canRollOut = true;
					}
					
					std::shared_ptr<RoadPoint> otherPoint = connection->connectedPoint;
					for (std::shared_ptr<Connection> otherConnection : otherPoint->connections) {
						if (otherConnection->connectedPoint == possibleStart && otherConnection->getOutputSpeed(MINIMUM_SPEED_KMH / 3.6) > 0) {
							canRollIn = true;
						}
					}
				}
				
				if (canRollOut && !canRollIn) {
					startingPoints.push_back(possibleStart);
				}
			}
			
			// Sort starting points by height
			sort(startingPoints.begin(), startingPoints.end(), [](std::shared_ptr<RoadPoint> point1, std::shared_ptr<RoadPoint> point2){return point1->z > point2->z;});
			
			int startingPointCounter = 0;
			while (startingPointCounter < startingPoints.size()) {
				std::shared_ptr<RoadPoint> currentPoint = startingPoints.at(startingPointCounter);
				std::shared_ptr<RouteGraph> routeGraph = std::shared_ptr<RouteGraph>(new RouteGraph(currentPoint));

				// Remove all starting points that can be reached from this initial starting point
				auto startingPointIterator = startingPoints.begin();
				while (startingPointIterator != startingPoints.end()) {
					if (routeGraph->reachablePoints.count(*startingPointIterator) > 0) {
						startingPointIterator = startingPoints.erase(startingPointIterator);
					} else {
						startingPointIterator++;
					}
				}
				
				startingPointCounter++;
				
				if (startingPointCounter % 100 == 0) {
					GeoDataDocument *document = new GeoDataDocument();
					addRouteGraphsToDocument(routeGraphs, document);
					emit mapUpdated(document, false, QString("Searching for possible starting points"), (int) (startingPointCounter * 100.0 / startingPoints.size()));
				}
				
				// Don't add starting point that don't go very far
				if (routeGraph->farthestDistance < MINIMUM_ROUTE_DISTANCE) {
					continue;
				}
				
				bool shouldInsert = true;
				
				auto routeGraphIterator = routeGraphs.begin();
				while (routeGraphIterator != routeGraphs.end()) {
					std::shared_ptr<RouteGraph> otherRouteGraph = *routeGraphIterator;			
					
					if (routeGraph->farthestEndpoint != otherRouteGraph->farthestEndpoint) {
						routeGraphIterator++;
						continue;
					}
					
					shouldInsert = false;
					
					// This seems to be have all the same endpoints as our current starting point
					// Check if this one is longer
					if (routeGraph->farthestDistance > otherRouteGraph->farthestDistance) {
						routeGraphIterator = routeGraphs.erase(routeGraphIterator);
						routeGraphIterator = routeGraphs.insert(routeGraphIterator, routeGraph);
					}
					
					break;
				}
				
				if (shouldInsert) {
					routeGraphs.push_back(routeGraph);
				}
			}
			
			emit mapUpdated(nullptr, true, QString("Finished searching for starting points"), 100);
			
			sort(this->routeGraphs.begin(), this->routeGraphs.end(), [](std::shared_ptr<RouteGraph> graph1, std::shared_ptr<RouteGraph> graph2){return graph1->farthestDistance > graph2->farthestDistance;});
			
			GeoDataDocument *document = new GeoDataDocument();
			addRouteGraphsToDocument(routeGraphs, document, 10);
			emit mapUpdated(document, false, QString("Showing 10 best starting points"), 0);
		}
		
		if (receivedClick) {
			// Find a route that is close enough
			std::shared_ptr<RouteGraph> closestRouteGraph = nullptr;
			for (auto routeGraph : routeGraphs) {
				if (abs(routeGraph->startingPoint->lat - clickedLat) < 0.00001 && abs(routeGraph->startingPoint->lon - clickedLon) < 0.00001) {
					closestRouteGraph = routeGraph;
					break;
				}
			}
			
			if (closestRouteGraph != nullptr) {
				GeoDataDocument *document = new GeoDataDocument();
				addRouteGraphsToDocument(routeGraphs, document, 10);
				addRouteGraphToDocument(closestRouteGraph, document);
				emit mapUpdated(document, false, QString("Showing possible roads"), 0);
			}
			
		}

		mutex.lock();
		condition.wait(&mutex);
		mutex.unlock();
	}
}